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
#include "debug/DifferentialMatchingPrefetcherRangeTableDebug.hh"
#include "mem/cache/prefetch/differential_matching_prefetcher/differential_matching_prefetcher_interface.hh"
#include "mem/cache/prefetch/differential_matching_prefetcher/prefetch_request.hh"
#include "mem/cache/prefetch/differential_matching_prefetcher/util.hh"

#define DMP_IRT_DEBUG(...) \
    DPRINTF(\
        DifferentialMatchingPrefetcherIndirectRelationTableDebug, \
        "%s: ", prefetcher_interface->getPrefetcherName().c_str()); \
    DPRINTFR(\
        DifferentialMatchingPrefetcherIndirectRelationTableDebug, \
        "(IRT) " __VA_ARGS__)

#define DMP_RT_DEBUG(...) \
    DPRINTF(\
        DifferentialMatchingPrefetcherRangeTableDebug, \
        "%s: ", prefetcher_interface->getPrefetcherName().c_str()); \
    DPRINTFR(\
        DifferentialMatchingPrefetcherRangeTableDebug, \
        "(RT) " __VA_ARGS__)

namespace gem5
{

namespace prefetch
{

namespace dmp
{

class RangeTableEntry
{
  public:
    Addr target_pc;
    uint64_t total_count;
    std::array<uint64_t, 9> range_counters;
    Addr prev_effective_address;
    Addr prev_access_size;
    uint64_t current_range_count;
    DifferentialMatchingPrefetcherInterface* prefetcher_interface;
  public:
    RangeTableEntry(
      const Addr _target_pc,
      DifferentialMatchingPrefetcherInterface* _prefetcher_interface
    );
    void profileL1CacheAccess(
        const Addr effective_address, const Addr size
    );
    uint64_t getPredictedRangeSize() const;
  private:
    void sampleRange(const uint64_t range_size);
    uint64_t getRangeBinWithMaxCount() const;
    uint64_t rangeSizeToBin(const uint64_t range_size) const;
    uint64_t binToPredictedRangeSize(const uint64_t bin) const;
};  // class RangeTableEntry

class IndirectRelationTableEntry
{
  private:
    static uint64_t next_id;
    uint64_t id;
    uint64_t cache_block_size;
    uint64_t block_shift;
  public:
    Addr index_pc;
    Addr target_pc;
    Addr target_base_vaddr;
    uint64_t shift_amount;
    AccessType index_access_type;
    AccessType target_access_type;
    RangeTableEntry range_table_entry;
    Tick prev_access_tick;
    DifferentialMatchingPrefetcherInterface* prefetcher_interface;
  public:
    IndirectRelationTableEntry(
        const Addr _index_pc,
        const Addr _target_pc,
        const Addr _target_base_vaddr,
        const uint64_t _shift_amount,
        const AccessType _index_access_type,
        const AccessType _target_access_type,
        const uint64_t _cache_block_size,
        DifferentialMatchingPrefetcherInterface* _prefetcher_interface
    );
    uint64_t getId() const;
    std::optional<std::vector<PrefetchRequest>> \
        getPrefetchesIfIndexPcMatches(
        const Addr index_pc, const int64_t data_from_index_pc
    );
  private:
    bool sameBlock(const Addr addr1, const Addr addr2) const;
};

class IndirectRelationTable
{
  private:
    const uint64_t max_num_indirect_relation_entries;
    const uint64_t max_num_range_table_entries;
    std::vector<IndirectRelationTableEntry> entries;
    const uint64_t cache_block_size;
    DifferentialMatchingPrefetcherInterface* prefetcher_interface;
  public:
    IndirectRelationTable(
      const uint64_t _max_num_indirect_relation_entries,
      const uint64_t _max_num_range_table_entries,
      const uint64_t _cache_block_size,
      DifferentialMatchingPrefetcherInterface* _prefetcher_interface
    );
    void addEntry(
        const Addr index_pc,
        const Addr target_pc,
        const Addr target_base_vaddr,
        const uint64_t shift_amount,
        const AccessType index_access_type,
        const AccessType target_access_type
    );
    bool isAnIndexPC(const Addr index_pc) const;
    bool isATargetPC(const Addr target_pc) const;
    bool containsEntry(const Addr index_pc, const Addr target_pc) const;
    std::optional<std::vector<PrefetchRequest>> queryEntryByIndexPc(
        const Addr index_pc, const int64_t data_from_index_pc
    );
    // Track L1 cache hit or miss accesses for tracking range accesses.
    void trackL1CacheAccess(
        const Addr target_pc, const Addr effective_address, const Addr size
    );
  private:
    bool isFull() const;
    uint64_t getCurrentNumRangeTableEntries() const;
    // Replace the least recently used entry in the table.
    // Note that, we have a small number of range table entries (which are part
    // of the indirect relation table entries), we use LRU as the replacement
    // policy with a small modification:
    //   - If we hit the capacity of range table entries, and the new entry
    // is a range type, we only replace an existing range type entry.
    //   - If the new entry is a single type, we can replace any existing
    // entry.
    void replaceLeastRecentlyUsedEntry(
        const Addr index_pc,
        const Addr target_pc,
        const Addr target_base_vaddr,
        const uint64_t shift_amount,
        const AccessType index_access_type,
        const AccessType target_access_type
    );
};

} // namespace dmp

} // namespace prefetch

} // namespace gem5

#endif // __DMP_INDIRECT_RELATION_TABLE_HH__
