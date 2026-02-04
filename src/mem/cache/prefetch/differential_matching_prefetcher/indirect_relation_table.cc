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

#include "mem/cache/prefetch/differential_matching_prefetcher/indirect_relation_table.hh"

#include <algorithm>
#include <cstdint>
#include <optional>
#include <vector>

#include "base/logging.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "debug/DifferentialMatchingPrefetcherIndirectRelationTableDebug.hh"
#include "mem/cache/prefetch/differential_matching_prefetcher/prefetch_request.hh"
#include "sim/cur_tick.hh"

namespace gem5
{

namespace prefetch
{

RangeTableEntry::RangeTableEntry(
  const Addr _target_pc
) : target_pc(_target_pc),
    total_count(0),
    prev_effective_address(0),
    prev_access_size(0),
    current_range_count(0)
{
    range_counters.fill(0);
}

void
RangeTableEntry::profileL1CacheAccess(
    const Addr effective_address, const Addr size
)
{
    if (effective_address == prev_effective_address) {
        // Duplicate access, ignore
        return;
    }
    if (effective_address == prev_effective_address +
        prev_access_size) {
        // Continuing the current range
        current_range_count += 1;
    } else {
        // New range detected
        if (current_range_count > 0) {
            sampleRange(current_range_count);
        }
        current_range_count = 1;
    }
    prev_effective_address = effective_address;
    prev_access_size = size;
}

uint64_t
RangeTableEntry::getPredictedRangeSize() const
{
    const uint64_t max_bin = getRangeBinWithMaxCount();
    return binToPredictedRangeSize(max_bin);
}

void
RangeTableEntry::sampleRange(const uint64_t range_size)
{
    total_count++;
    range_counters[rangeSizeToBin(range_size)]++;
    DMP_RT_DEBUG(
        "RangeTableEntry Target PC %#x: Starting from vaddr %#x, "
        "sampled range size %lu, updated bin %lu count to %lu\n",
        target_pc,
        prev_effective_address - prev_access_size * (current_range_count - 1),
        range_size,
        rangeSizeToBin(range_size),
        range_counters[rangeSizeToBin(range_size)]
    );
}

uint64_t
RangeTableEntry::getRangeBinWithMaxCount() const
{
    uint64_t max_count = 0;
    uint64_t max_bin = 0;
    for (uint64_t bin = 0; bin < range_counters.size(); bin++) {
        // pick the highest bin in case of tie
        if (range_counters[bin] >= max_count) {
            max_count = range_counters[bin];
            max_bin = bin;
        }
    }
    return max_bin;
}

uint64_t
RangeTableEntry::rangeSizeToBin(const uint64_t range_size) const
{
    switch (range_size) {
        case 1:
        case 2:
            return 0;
        case 3:
        case 4:
            return 1;
        case 5:
        case 6:
            return 2;
        case 7:
        case 8:
            return 3;
        case 9:
        case 10:
            return 4;
        case 11:
        case 12:
            return 5;
        case 13:
        case 14:
            return 6;
        case 15:
        case 16:
            return 7;
        default:
            return 8;
    }
    return 8; // Should not reach here
}

uint64_t
RangeTableEntry::binToPredictedRangeSize(const uint64_t bin) const
{
    switch (bin) {
        case 0:
            return 2;
        case 1:
            return 4;
        case 2:
            return 6;
        case 3:
            return 8;
        case 4:
            return 10;
        case 5:
            return 12;
        case 6:
            return 14;
        case 7:
            return 16;
        default:
            return 32; // For bin 8 and above
    }
    return 32; // Should not reach here
}

IndirectRelationTableEntry::IndirectRelationTableEntry(
  const Addr _index_pc,
  const Addr _target_pc,
  const Addr _target_base_vaddr,
  const uint64_t _shift_amount,
  const AccessType _index_access_type,
  const AccessType _target_access_type
) : id(next_id++),
    index_pc(_index_pc),
    target_pc(_target_pc),
    target_base_vaddr(_target_base_vaddr),
    shift_amount(_shift_amount),
    index_access_type(_index_access_type),
    target_access_type(_target_access_type),
    range_table_entry(_target_pc),
    prev_access_tick(curTick())
{
}

uint64_t
IndirectRelationTableEntry::getId() const
{
    return id;
}

std::optional<std::vector<DMPPrefetchRequest>>
IndirectRelationTableEntry::getPrefetchesIfIndexPcMatches(
    const Addr index_pc, const int64_t data_from_index_pc
)
{
    if (index_pc == this->index_pc) {
        prev_access_tick = curTick();
        std::vector<DMPPrefetchRequest> prefetch_requests;
        // Generate prefetch requests based on data_from_index_pc
        if (target_access_type == AccessType::Single) {
            Addr prefetch_vaddr = target_base_vaddr +
                (data_from_index_pc << shift_amount);
            prefetch_requests.emplace_back(
                /*target_pc*/ target_pc,
                /*prefetch_vaddr*/ prefetch_vaddr,
                /*size*/ 1ULL << shift_amount,
                /*irt_id*/ id
            );
        } else if (target_access_type == AccessType::Range) {
            // For range access, we can prefetch a range of addresses
            DMP_IRT_DEBUG(
                "Range access type is not yet implemented in "
                "IndirectRelationTableEntry::getPrefetchesIfIdMatches.\n"
            );
        }
        return prefetch_requests;
    }
    return std::nullopt;
}

IndirectRelationTable::IndirectRelationTable(
  const uint64_t _max_num_indirect_relation_entries,
  const uint64_t _max_num_range_table_entries
) : max_num_indirect_relation_entries(_max_num_indirect_relation_entries),
    max_num_range_table_entries(_max_num_range_table_entries),
    entries()
{
    entries.reserve(max_num_indirect_relation_entries);
}

uint64_t IndirectRelationTableEntry::next_id = 0;

std::optional<std::vector<DMPPrefetchRequest>>
IndirectRelationTable::queryEntryByIndexPc(
    const Addr index_pc, const int64_t data_from_index_pc
)
{
    for (auto &entry : entries) {
        if (entry.index_pc == index_pc) {
            return entry.getPrefetchesIfIndexPcMatches(
                index_pc, data_from_index_pc
            );
        }
    }
    return std::nullopt;
}

void
IndirectRelationTable::addEntry(
  const Addr index_pc,
  const Addr target_pc,
  const Addr target_base_vaddr,
  const uint64_t shift_amount,
  const AccessType index_access_type,
  const AccessType target_access_type
)
{
    if (containsEntry(index_pc, target_pc)) {
        DMP_IRT_DEBUG(
            "IndirectRelationTable already contains entry: Index PC %#x, "
            "Target PC %#x\n",
            index_pc, target_pc
        );
        return;
    }

    // If the table is full, we need to replace an existing entry.
    // If we have reached the capacity of range table entries, and the new
    // entry is a range type, we only consider replacing existing range type
    // entries.
    if (isFull() || (target_access_type == AccessType::Range &&
        getCurrentNumRangeTableEntries() >= max_num_range_table_entries)) {
        replaceLeastRecentlyUsedEntry(
            index_pc, target_pc, target_base_vaddr, shift_amount,
            index_access_type, target_access_type
        );
    } else {
        entries.emplace_back(
            index_pc, target_pc, target_base_vaddr, shift_amount,
            index_access_type, target_access_type
        );
    }

    const IndirectRelationTableEntry &entry = entries.back();
    DMP_IRT_DEBUG(
        "Added IndirectRelationTableEntry ID %llu: Index PC %#x, "
        "Target PC %#x, Target Base Vaddr %#x, Shift Amount %llu, "
        "Index Access Type %s, Target Access Type %s, "
        "Previous Access Tick %llu\n",
        entry.getId(),
        entry.index_pc,
        entry.target_pc,
        entry.target_base_vaddr,
        entry.shift_amount,
        (entry.index_access_type == AccessType::Single) ?
                                                        "Single" : "Range",
        (entry.target_access_type == AccessType::Single) ?
                                                        "Single" : "Range",
        entry.prev_access_tick
    );
}

bool
IndirectRelationTable::containsEntry(
    const Addr index_pc, const Addr target_pc
) const
{
    for (const auto &entry : entries) {
        if (entry.index_pc == index_pc && entry.target_pc == target_pc) {
            return true;
        }
    }
    return false;
}

void
IndirectRelationTable::trackL1CacheAccess(
    const Addr target_pc, const Addr effective_address, const Addr size
)
{
    for (auto &entry : entries) {
        if (entry.target_pc == target_pc &&
            entry.target_access_type == AccessType::Range) {
            // For range type entries, we track L1 cache accesses
            entry.range_table_entry.profileL1CacheAccess(
                effective_address, size
            );
        }
    }
}

bool
IndirectRelationTable::isFull() const
{
    return entries.size() >= max_num_indirect_relation_entries;
}

uint64_t
IndirectRelationTable::getCurrentNumRangeTableEntries() const
{
    uint64_t count = 0;
    for (const auto &entry : entries) {
        if (entry.target_access_type == AccessType::Range) {
            count++;
        }
    }
    return count;
}

void
IndirectRelationTable::replaceLeastRecentlyUsedEntry(
  const Addr index_pc,
  const Addr target_pc,
  const Addr target_base_vaddr,
  const uint64_t shift_amount,
  const AccessType index_access_type,
  const AccessType target_access_type
)
{
    // If we have reached the capacity of range table entries, and the new
    // entry is a range type, we only consider replacing existing range type
    // entries.
    // Find the least recently used entry
    auto lru_it = entries.begin();

    if (getCurrentNumRangeTableEntries() >= max_num_range_table_entries &&
        target_access_type == AccessType::Range) {
        Tick min_prev_access_tick = std::numeric_limits<Tick>::max();
        for (auto it = entries.begin(); it != entries.end(); ++it) {
            if (it->target_access_type == AccessType::Range &&
                it->prev_access_tick < min_prev_access_tick) {
                min_prev_access_tick = it->prev_access_tick;
                lru_it = it;
            }
        }
    } else {
        // Otherwise, we can replace any entry
        lru_it = std::min_element(
            entries.begin(), entries.end(),
            [](const IndirectRelationTableEntry &a,
               const IndirectRelationTableEntry &b) {
                return a.prev_access_tick < b.prev_access_tick;
            }
        );
    }
    if (lru_it != entries.end()) {
        DMP_IRT_DEBUG(
            "Replacing least recently used IndirectRelationTableEntry: "
            "previously accessed %llu\n",
            lru_it->prev_access_tick
        );
        *lru_it = IndirectRelationTableEntry(
            index_pc, target_pc, target_base_vaddr, shift_amount,
            index_access_type, target_access_type
        );
    }
}


} // namespace prefetch

} // namespace gem5
