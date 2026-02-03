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

#include <sstream>
#include <utility>

#include "base/trace.hh"
#include "base/types.hh"
#include "debug/DifferentialMatchingPrefetcherDifferentMatcherDebug.hh"
#include "mem/cache/prefetch/differential_matching_prefetcher/differential_matching_prefetcher_interface.hh"

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

TrackingEntryWithRepetitionFilterAndRangeFilter::\
    TrackingEntryWithRepetitionFilterAndRangeFilter (
    const Addr _pc,
    const uint64_t _max_num_tracked_items
) : pc(_pc),
    max_num_tracked_items(_max_num_tracked_items),
    previous_tracked_item(0),
    previous_size(0)
{
    tracked_items.reserve(max_num_tracked_items);
}

void
TrackingEntryWithRepetitionFilterAndRangeFilter::addItem(
    const Addr item, const uint64_t size
)
{
    // If we have already tracked the maximum number of items, we do not add
    // more
    if (tracked_items.size() >= max_num_tracked_items) {
        return;
    }
    if (!tracked_items.empty()) {
        // If the new item is exactly the same as the previous tracked item,
        // we do not add it (repetition filter)
        if (tracked_items.back().first == item) {
            return;
        }
        // If the new item is exactly after the previous tracked item, we
        // update the size of the last tracked item (range filter)
        if (previous_tracked_item + previous_size == item) {
            // The previous_size should be equal to size
            // assert(previous_size == size);
            // Increase the range size of the last tracked item
            tracked_items.back().second += 1;
            previous_tracked_item = item;
            return;
        }
    }
    tracked_items.emplace_back(item, 1);
    previous_tracked_item = item;
    previous_size = size;
}

bool
TrackingEntryWithRepetitionFilterAndRangeFilter::isFull() const
{
    return tracked_items.size() >= max_num_tracked_items;
}


DifferentialMatcher::DifferentialMatcher(
    const uint64_t _max_num_index_table_entries,
    const uint64_t _max_num_tracked_items_per_index_table_entry,
    const uint64_t _max_num_target_table_entries,
    const uint64_t _max_num_tracked_items_per_target_table_entry,
    const std::vector<int64_t> &_matching_shift_amounts,
    DifferentialMatchingPrefetcherInterface *_prefetcher_interface
) : max_num_index_table_entries(_max_num_index_table_entries),
    max_num_tracked_items_per_index_table_entry(
        _max_num_tracked_items_per_index_table_entry
    ),
    max_num_target_table_entries(_max_num_target_table_entries),
    max_num_tracked_items_per_target_table_entry(
        _max_num_tracked_items_per_target_table_entry
    ),
    matching_shift_amounts(_matching_shift_amounts),
    prefetcher_interface(_prefetcher_interface)
{
    panic_if(
        max_num_index_table_entries != max_num_target_table_entries,
        "For simplicity, the maximum number of entries in index and target "
        "tables must be the same."
    );
    panic_if(
        _max_num_tracked_items_per_target_table_entry <= 3,
        "We expecet to have minimum 4 tracked items per target table entry "
        "to produce at least 3 diffs."
    );
    panic_if(
        matching_shift_amounts.empty(),
        "At least one matching shift amount must be provided."
    );
}

bool
DifferentialMatcher::isEmpty() const
{
    return candidate_index_target_pc.empty();
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
        IndexPcTrackingEntry(
            index_pc, max_num_tracked_items_per_index_table_entry
        ),
        TargetPcTrackingEntry(
            target_pc, max_num_tracked_items_per_target_table_entry
        )
    );
    candidate_index_target_pc.emplace(pc_pair, tracking_pair);
    DMP_DIFFERENTIAL_MATCHER_DEBUG(
        "New candidate pair added: Index PC %#x, Target PC %#x\n",
        index_pc, target_pc
    );
    return true;
}

void
DifferentialMatcher::trackL1CacheHit(
    const Addr pc, const Addr effective_vaddr, const uint64_t data,
    const uint64_t request_size
)
{
    bool has_full_target_entry = false;

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
            if (target_entry.isFull()) {
                has_full_target_entry = true;
            }
        }
    }

    // If any target entry is full, we try to match candidates
    if (has_full_target_entry) {
        tryMatchingCandidates();
    }
}

void
DifferentialMatcher::trackL1CacheMiss(
    const Addr pc, const Addr effective_vaddr, const uint64_t request_size
)
{
    bool has_full_target_entry = false;

    // We only track target PC cache misses
    for (
        auto &[candidate_pair, tracking_entries] : candidate_index_target_pc
    ) {
        const Addr target_pc = candidate_pair.second;
        TargetPcTrackingEntry &target_entry = tracking_entries.second;
        if (pc == target_pc) {
            // This is a target PC cache miss
            target_entry.addItem(effective_vaddr, request_size);
            if (target_entry.isFull()) {
                has_full_target_entry = true;
            }
        }
    }

    // If any target entry is full, we try to match candidates
    if (has_full_target_entry) {
        tryMatchingCandidates();
    }
}

void
DifferentialMatcher::trackL1CacheFill(
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
    const auto &target_filtered_items = target_entry.tracked_items;
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

    std::stringstream index_strm;
    index_strm << "Index tracked items: ";
    for (const auto &item : index_entry.tracked_items) {
        index_strm << std::hex << "0x" << item.first << " " << std::dec;
    }
    DMP_DIFFERENTIAL_MATCHER_DEBUG("%s\n", index_strm.str().c_str());

    std::stringstream target_strm;
    target_strm << "Target tracked items: ";
    for (const auto &item : target_filtered_items) {
        target_strm << std::hex << "(0x" << item.first << ", " << std::dec <<
            item.second << ") ";
    }
    DMP_DIFFERENTIAL_MATCHER_DEBUG("%s\n", target_strm.str().c_str());

    std::stringstream index_diff_strm;
    index_diff_strm << "Index diffs: ";
    for (const auto &diff : index_diffs) {
        index_diff_strm << diff << " ";
    }
    DMP_DIFFERENTIAL_MATCHER_DEBUG("%s\n", index_diff_strm.str().c_str());

    std::stringstream target_diff_strm;
    target_diff_strm << "Target diffs: ";
    for (const auto &diff : target_diffs) {
        target_diff_strm << diff << " ";
    }
    DMP_DIFFERENTIAL_MATCHER_DEBUG("%s\n", target_diff_strm.str().c_str());

    // For each shift amount, we form groups of 3 of index data diffs to match
    // with target diffs
    bool match_found = false;
    int64_t match_shift_amount_index = 0;
    Addr target_base_vaddr = 0xBADC0FFEE; // Placeholder
    for (const auto &shift_amount : matching_shift_amounts) {

        const std::vector<int64_t> shifted_index_diffs = (shift_amount > 0) ?
            multiplyVectorByFactor(index_diffs, 1LL << shift_amount) : \
            multiplyVectorByFactor(index_diffs, (-1LL) << shift_amount);
        DMP_DIFFERENTIAL_MATCHER_DEBUG(
            "Matching with shift amount %lld:\n", shift_amount
        );
        // Perform matching between index_diffs and shifted_target_diffs
        for (uint64_t i = 0; i + 2 < shifted_index_diffs.size(); ++i) {
            int64_t idx_diff1 = shifted_index_diffs[i];
            int64_t idx_diff2 = shifted_index_diffs[i + 1];
            int64_t idx_diff3 = shifted_index_diffs[i + 2];
            for (uint64_t j = 0; j + 2 < target_diffs.size(); ++j) {
                int64_t tgt_diff1 = target_diffs[j];
                int64_t tgt_diff2 = target_diffs[j + 1];
                int64_t tgt_diff3 = target_diffs[j + 2];
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
                    target_base_vaddr =
                        target_filtered_items[j].first -
                        shifted_index_diffs[i];
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

    DMP_DIFFERENTIAL_MATCHER_DEBUG(
        "Max range counter for target PC %#x: %llu\n",
        target_pc, max_range_counter
    );

    // Notify the prefetcher of the match result,
    prefetcher_interface->handleDifferentialMatchResult(
        /*index_pc*/ index_pc, /*target_pc*/ target_pc,
        /*match_found*/ match_found,
        /*target_base_vaddr*/ target_base_vaddr,
        /*shift_amount*/ match_found ? match_shift_amount_index : 0,
        /*index_access_type*/ AccessType::Single,
        /*target_access_type*/ (max_range_counter == 1) ?
            AccessType::Single : AccessType::Range
    );
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

void
DifferentialMatcher::tryMatchingCandidates()
{
    // We attempt to match when the target entry is full
    std::vector<CandidatePcPair> candidates_to_remove;
    for (
        auto &[candidate_pair, tracking_entries] : candidate_index_target_pc
    ) {
        const Addr index_pc = candidate_pair.first;
        const Addr target_pc = candidate_pair.second;
        TargetPcTrackingEntry &target_entry = tracking_entries.second;
        if (target_entry.isFull()) {
            // Attempt to match this candidate pair
            matchCandidate(index_pc, target_pc);
            // After matching, we remove this candidate pair from tracking
            candidates_to_remove.push_back(candidate_pair);
        }
    }

    // Remove the matched candidates from tracking
    for (const auto &candidate_pair : candidates_to_remove) {
        candidate_index_target_pc.erase(candidate_pair);
        DMP_DIFFERENTIAL_MATCHER_DEBUG(
            "Removed candidate pair: (%#x, %#x)\n",
            candidate_pair.first, candidate_pair.second
        );
    }
}

} // namespace prefetch

} // namespace gem5
