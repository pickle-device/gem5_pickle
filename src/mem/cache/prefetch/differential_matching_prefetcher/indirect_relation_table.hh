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

#ifndef __DMP_INDIRECT_RELATION_TABLE_HH__
#define __DMP_INDIRECT_RELATION_TABLE_HH__

#include <cstdint>
#include <optional>
#include <vector>

#include "base/logging.hh"
#include "base/types.hh"
#include "debug/DifferentialMatchingPrefetcherIndirectRelationTableDebug.hh"
#include "mem/cache/prefetch/differential_matching_prefetcher/differential_matching_prefetcher_interface.hh"
#include "mem/cache/prefetch/differential_matching_prefetcher/prefetch_request.hh"

#define DMP_IRT_DEBUG(...) \
    DPRINTF(\
        DifferentialMatchingPrefetcherIndirectRelationTableDebug, \
        "(IRT) " __VA_ARGS__)

namespace gem5
{

namespace prefetch
{

class IndirectRelationTableEntry
{
  private:
    static uint64_t next_id;
    uint64_t id;
  public:
    Addr index_pc;
    Addr target_pc;
    Addr target_base_vaddr;
    uint64_t shift_amount;
    AccessType index_access_type;
    AccessType target_access_type;
    // std::shared_ptr<RangeTable> range_table; // TODO: implement RangeTable
    Tick prev_access_tick;
  public:
    IndirectRelationTableEntry(
        const Addr _index_pc,
        const Addr _target_pc,
        const Addr _target_base_vaddr,
        const uint64_t _shift_amount,
        const AccessType _index_access_type,
        const AccessType _target_access_type
    );
    uint64_t getId() const;
    std::optional<std::vector<DMPPrefetchRequest>> \
        getPrefetchesIfIndexPcMatches(
        const Addr index_pc, const int64_t data_from_index_pc
    );
};

class IndirectRelationTable
{
  private:
    const uint64_t max_num_entries;
    std::vector<IndirectRelationTableEntry> entries;
  public:
    IndirectRelationTable(const uint64_t _max_num_entries);
    void addEntry(
        const Addr index_pc,
        const Addr target_pc,
        const Addr target_base_vaddr,
        const uint64_t shift_amount,
        const AccessType index_access_type,
        const AccessType target_access_type
    );
    bool containsEntry(const Addr index_pc, const Addr target_pc) const;
    std::optional<std::vector<DMPPrefetchRequest>> queryEntryByIndexPc(
        const Addr index_pc, const int64_t data_from_index_pc
    );
  private:
    bool isFull() const;
    void replaceLeastRecentlyUsedEntry(
        const Addr index_pc,
        const Addr target_pc,
        const Addr target_base_vaddr,
        const uint64_t shift_amount,
        const AccessType index_access_type,
        const AccessType target_access_type
    );
};

} // namespace prefetch

} // namespace gem5

#endif // __DMP_INDIRECT_RELATION_TABLE_HH__
