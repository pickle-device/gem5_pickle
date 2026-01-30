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
  const uint64_t _max_num_entries
) : max_num_entries(_max_num_entries),
    entries()
{
    entries.reserve(max_num_entries);
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
    if (isFull()) {
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

    for (const auto &entry : entries) {
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

}

bool
IndirectRelationTable::isFull() const
{
    return entries.size() >= max_num_entries;
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
    // Find the least recently used entry
    auto lru_it = std::min_element(
        entries.begin(), entries.end(),
        [](const IndirectRelationTableEntry &a,
           const IndirectRelationTableEntry &b) {
            return a.prev_access_tick < b.prev_access_tick;
        }
    );
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
