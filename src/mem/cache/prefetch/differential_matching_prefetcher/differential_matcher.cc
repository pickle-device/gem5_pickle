/*
 * Copyright (c) 2026 The Regents of the University of California
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "mem/cache/prefetch/differential_matching_prefetcher/differential_matcher.hh"

#include <utility>

#include "base/trace.hh"
#include "base/types.hh"
#include "debug/DifferentialMatchingPrefetcherDifferentMatcherDebug.hh"

namespace gem5
{

namespace prefetch
{

TrackingEntryWithRepetitionFilter::TrackingEntryWithRepetitionFilter(
    const Addr _pc,
    const uint64_t _max_num_tracked_items
) : pc(_pc),
    max_num_tracked_items(_max_num_tracked_items)
{
    tracked_items.reserve(max_num_tracked_items);
}

void
TrackingEntryWithRepetitionFilter::addItem(
    const Addr item, const uint64_t size
)
{
    // If we have already tracked the maximum number of items, we do not add
    // more
    if (tracked_items.size() >= max_num_tracked_items) {
        return;
    }
    if (!tracked_items.empty()) {
        // Check for repetition with the last tracked item
        if (tracked_items.back().first == item) {
            return; // Do not add repeated item
        }
    }
    tracked_items.emplace_back(item, size);
}

bool
TrackingEntryWithRepetitionFilter::isFull() const
{
    return tracked_items.size() >= max_num_tracked_items;
}

std::vector<std::pair<Addr, uint64_t>>
TrackingEntryWithRepetitionFilter::getRangeFilteredItems() const
{
    std::vector<std::pair<Addr, uint64_t>> filtered_items;
    if (tracked_items.empty()) {
        return filtered_items;
    }
    // Apply range filtering to the tracked items
    Addr range_start = tracked_items.front().first;
    uint64_t range_size = 0;
    for (const auto &[item, size] : tracked_items) {
        if (item == range_start + range_size) {
            range_size += size;
        } else {
            filtered_items.emplace_back(range_start, range_size);
            range_start = item;
            range_size = size;
        }
    }
    // Add the last range
    filtered_items.emplace_back(range_start, range_size);
    return filtered_items;
}

DifferentialMatcher::DifferentialMatcher(
    const uint64_t _max_num_index_table_entries,
    const uint64_t _max_num_target_table_entries,
    const uint64_t _max_num_tracked_items_per_table_entry,
    const std::vector<uint64_t> &_matching_shift_amounts
) : max_num_index_table_entries(_max_num_index_table_entries),
    max_num_target_table_entries(_max_num_target_table_entries),
    max_num_tracked_items_per_table_entry(
        _max_num_tracked_items_per_table_entry
    ),
    matching_shift_amounts(_matching_shift_amounts)
{
    panic_if(
        max_num_index_table_entries != max_num_target_table_entries,
        "For simplicity, the maximum number of entries in index and target "
        "tables must be the same."
    );
    panic_if(
        matching_shift_amounts.empty(),
        "At least one matching shift amount must be provided."
    );
}

bool
DifferentialMatcher::isFull() const
{
    return candidate_index_target_pc.size() >= max_num_index_table_entries;
}

bool
DifferentialMatcher::hasCandidate(
    const Addr index_pc, const Addr target_pc
) const
{
    CandidatePcPair pc_pair = std::make_pair(index_pc, target_pc);
    return candidate_index_target_pc.find(pc_pair) !=
        candidate_index_target_pc.end();
}

bool
DifferentialMatcher::addCandidate(const Addr index_pc, const Addr target_pc)
{
    if (isFull() || hasCandidate(index_pc, target_pc)) {
        return false; // Cannot add new candidate
    }
    CandidatePcPair pc_pair = std::make_pair(index_pc, target_pc);
    TrackingPair tracking_pair = std::make_pair(
        IndexPcTrackingEntry(index_pc, max_num_tracked_items_per_table_entry),
        TargetPcTrackingEntry(target_pc, max_num_tracked_items_per_table_entry)
    );
    candidate_index_target_pc.emplace(pc_pair, tracking_pair);
    DMP_DIFFERENTIAL_MATCHER_DEBUG(
        "New candidate pair added: Index PC %#x, Target PC %#x\n",
        index_pc, target_pc
    );
    return true;
}

void
DifferentialMatcher::trackCacheHit(
    const Addr pc, const Addr effective_vaddr, const uint64_t data,
    const uint64_t request_size
)
{
    // We track index PC hits with data, and target PC hits with effective
    // virtual addresses.
    for (
        auto &[candidate_pair, tracking_entries] : candidate_index_target_pc
    ) {
        const Addr index_pc = candidate_pair.first;
        const Addr target_pc = candidate_pair.second;
        IndexPcTrackingEntry &index_entry = tracking_entries.first;
        TargetPcTrackingEntry &target_entry = tracking_entries.second;
        if (pc == index_pc) {
            // This is an index PC cache hit
            index_entry.addItem(data, request_size);
        }
        if (pc == target_pc) {
            // This is a target PC cache hit
            target_entry.addItem(effective_vaddr, request_size);
        }
    }
}

void
DifferentialMatcher::trackCacheMiss(
    const Addr pc, const Addr effective_vaddr, const uint64_t request_size
)
{
    // We only track target PC cache misses
    for (
        auto &[candidate_pair, tracking_entries] : candidate_index_target_pc
    ) {
        const Addr target_pc = candidate_pair.second;
        TargetPcTrackingEntry &target_entry = tracking_entries.second;
        if (pc == target_pc) {
            // This is a target PC cache miss
            target_entry.addItem(effective_vaddr, request_size);
        }
    }
}

void
DifferentialMatcher::trackCacheFill(
    const Addr pc, const Addr effective_vaddr, const uint64_t data,
    const uint64_t request_size
)
{
    // We only track index PC cache fills with data
    for (
        auto &[candidate_pair, tracking_entries] : candidate_index_target_pc
    ) {
        const Addr index_pc = candidate_pair.first;
        IndexPcTrackingEntry &index_entry = tracking_entries.first;
        if (pc == index_pc) {
            // This is an index PC cache fill
            index_entry.addItem(data, request_size);
        }
    }
}

void
DifferentialMatcher::matchCandidate(
    const Addr index_pc, const Addr target_pc
) const
{
    CandidatePcPair pc_pair = std::make_pair(index_pc, target_pc);
    auto it = candidate_index_target_pc.find(pc_pair);
    if (it == candidate_index_target_pc.end()) {
        DMP_DIFFERENTIAL_MATCHER_DEBUG(
            "No candidate pair found for Index PC %#x, Target PC %#x\n",
            index_pc, target_pc
        );
        return;
    }
    const TrackingPair &tracking_entries = it->second;
    const IndexPcTrackingEntry &index_entry = tracking_entries.first;
    const TargetPcTrackingEntry &target_entry = tracking_entries.second;

    DMP_DIFFERENTIAL_MATCHER_DEBUG(
        "Matching candidate pair: Index PC %#x, Target PC %#x\n",
        index_pc, target_pc
    );

    // Compute the differences for index and target tracked items
    std::vector<int64_t> index_diffs;
    for (size_t i = 1; i < index_entry.tracked_items.size(); ++i) {
        const int64_t curr_data = static_cast<int64_t>(
            index_entry.tracked_items[i].first
        );
        const int64_t prev_data = static_cast<int64_t>(
            index_entry.tracked_items[i - 1].first
        );
        const int64_t diff = curr_data - prev_data;
        index_diffs.push_back(diff);
    }

    std::vector<int64_t> target_diffs;
    std::vector<std::pair<Addr, uint64_t>> target_filtered_items =
        target_entry.getRangeFilteredItems();
    const size_t max_range_counter = std::max_element(
        target_filtered_items.begin(),
        target_filtered_items.end(),
        [](const auto &a, const auto &b) {
            return a.second < b.second;
        }
    )->second;
    for (size_t i = 1; i < target_filtered_items.size(); ++i) {
        const int64_t curr_eff_addr = static_cast<int64_t>(
            target_filtered_items[i].first
        );
        const int64_t prev_eff_addr = static_cast<int64_t>(
            target_filtered_items[i - 1].first
        );
        const int64_t diff = curr_eff_addr - prev_eff_addr;
        target_diffs.push_back(diff);
    }

    DMP_DIFFERENTIAL_MATCHER_DEBUG(
        "Index diffs: "
    );
    for (const auto &diff : index_diffs) {
        DMP_DIFFERENTIAL_MATCHER_DEBUG("%lld ", diff);
    }
    DMP_DIFFERENTIAL_MATCHER_DEBUG("\n");
    DMP_DIFFERENTIAL_MATCHER_DEBUG(
        "Target diffs: "
    );
    for (const auto &diff : target_diffs) {
        DMP_DIFFERENTIAL_MATCHER_DEBUG("%lld ", diff);
    }
    DMP_DIFFERENTIAL_MATCHER_DEBUG("\n");

    // For each shift amount, we form groups of 3 of index data diffs to match
    // with target diffs
    bool match_found = false;
    int64_t match_shift_amount_index = 0;
    for (const auto &shift_amount : matching_shift_amounts) {
        const std::vector<int64_t> shifted_target_diffs =
            multiplyVectorByFactor(target_diffs, 1LL << shift_amount);
        DMP_DIFFERENTIAL_MATCHER_DEBUG(
            "Matching with shift amount %llu:\n", shift_amount
        );
        // Perform matching between index_diffs and shifted_target_diffs
        for (uint64_t i = 0; i + 2 < index_diffs.size(); ++i) {
            int64_t idx_diff1 = index_diffs[i];
            int64_t idx_diff2 = index_diffs[i + 1];
            int64_t idx_diff3 = index_diffs[i + 2];
            for (uint64_t j = 0; j + 2 < shifted_target_diffs.size(); ++j) {
                int64_t tgt_diff1 = shifted_target_diffs[j];
                int64_t tgt_diff2 = shifted_target_diffs[j + 1];
                int64_t tgt_diff3 = shifted_target_diffs[j + 2];
                if (idx_diff1 == tgt_diff1 &&
                    idx_diff2 == tgt_diff2 &&
                    idx_diff3 == tgt_diff3) {
                    DMP_DIFFERENTIAL_MATCHER_DEBUG(
                        "Match found: Index diffs (%lld, %lld, %lld) "
                        "with Target diffs (%lld, %lld, %lld)\n",
                        idx_diff1, idx_diff2, idx_diff3,
                        tgt_diff1, tgt_diff2, tgt_diff3
                    );
                    match_found = true;
                    match_shift_amount_index = shift_amount;
                    break;
                }
            }
            if (match_found) {
                break;
            }
        }
        if (match_found) {
            break;
        }
    }
    if (!match_found) {
        DMP_DIFFERENTIAL_MATCHER_DEBUG("No match found.\n");
    }

    // TODO: notify the prefetcher of the match result, add a new interface
}

std::vector<int64_t>
DifferentialMatcher::multiplyVectorByFactor(
      const std::vector<int64_t> &vec, const int64_t factor
    ) const
{
    std::vector<int64_t> result;
    result.reserve(vec.size());
    for (const auto &val : vec) {
        result.push_back(val * factor);
    }
    return result;
}

} // namespace prefetch

} // namespace gem5
