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

namespace gem5
{

namespace prefetch
{

TrackingEntry::TrackingEntry(
    const Addr _pc,
    const uint64_t _max_num_tracked_items
) : pc(_pc),
    max_num_tracked_items(_max_num_tracked_items)
{
    tracked_items.reserve(max_num_tracked_items);
}

void TrackingEntry::addItem(const Addr item)
{
    // If we have already tracked the maximum number of items, we do not add
    // more
    if (tracked_items.size() >= max_num_tracked_items) {
        return;
    }
    tracked_items.push_back(item);
}

bool
TrackingEntry::isFull() const
{
    return tracked_items.size() >= max_num_tracked_items;
}

DifferentialMatcher::DifferentialMatcher(
    const uint64_t _max_num_index_table_entries,
    const uint64_t _max_num_target_table_entries,
    const uint64_t _max_num_tracked_items_per_table_entry
) : max_num_index_table_entries(_max_num_index_table_entries),
    max_num_target_table_entries(_max_num_target_table_entries),
    max_num_tracked_items_per_table_entry(
        _max_num_tracked_items_per_table_entry
    )
{
    panic_if(
        max_num_index_table_entries != max_num_target_table_entries,
        "For simplicity, the maximum number of entries in index and target "
        "tables must be the same."
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
    const Addr pc, const Addr effective_vaddr, const uint64_t data
)
{
    // We track index PC hits with data, and target PC hits with effective
    // virtual addresses.
    for (
        auto &[candidate_pair, tracking_entries] : candidate_index_target_pc
    ) {
        const Addr index_pc = candidate_pair.first;
        const Addr target_pc = candidate_pair.second;
        TrackingEntry &index_entry = tracking_entries.first;
        TrackingEntry &target_entry = tracking_entries.second;
        if (pc == index_pc) {
            // This is an index PC cache hit
            index_entry.addItem(data);
        }
        if (pc == target_pc) {
            // This is a target PC cache hit
            target_entry.addItem(effective_vaddr);
        }
    }
}

void
DifferentialMatcher::trackCacheMiss(
    const Addr pc, const Addr effective_vaddr
)
{
    // We only track target PC cache misses
    for (
        auto &[candidate_pair, tracking_entries] : candidate_index_target_pc
    ) {
        const Addr target_pc = candidate_pair.second;
        TrackingEntry &target_entry = tracking_entries.second;
        if (pc == target_pc) {
            // This is a target PC cache miss
            target_entry.addItem(effective_vaddr);
        }
    }
}

void
DifferentialMatcher::trackCacheFill(
    const Addr pc, const Addr effective_vaddr, const uint64_t data
)
{
    // We only track index PC cache fills with data
    for (
        auto &[candidate_pair, tracking_entries] : candidate_index_target_pc
    ) {
        const Addr index_pc = candidate_pair.first;
        TrackingEntry &index_entry = tracking_entries.first;
        if (pc == index_pc) {
            // This is an index PC cache fill
            index_entry.addItem(data);
        }
    }
}

} // namespace prefetch

} // namespace gem5
