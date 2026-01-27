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

#ifndef __DMP_DIFFERENTIAL_MATCHER_HH__
#define __DMP_DIFFERENTIAL_MATCHER_HH__

#include <cstdint>
#include <map>
#include <unordered_map>
#include <utility>
#include <vector>

#include "base/logging.hh"
#include "base/types.hh"
#include "debug/DifferentialMatchingPrefetcherDifferentMatcherDebug.hh"

#define DMP_DIFFERENTIAL_MATCHER_DEBUG(...) \
    DPRINTF(DifferentialMatchingPrefetcherDifferentMatcherDebug, __VA_ARGS__)

namespace gem5
{

namespace prefetch
{

/*
 * Data structures and methods for performing differential matching between
 * index PC and target PC.
 */

class TrackingEntry
{
  public:
    Addr pc;
    uint64_t max_num_tracked_items;
    std::vector<Addr> tracked_items;
    TrackingEntry(
      const Addr _pc, const uint64_t _max_num_tracked_items
    );
    void addItem(const Addr item);
    bool isFull() const;
}; // class TrackingEntry

class DifferentialMatcher
{
  private:
    // Parameters
    const uint64_t max_num_index_table_entries;
    const uint64_t max_num_target_table_entries;
    const uint64_t max_num_tracked_items_per_table_entry;
  public:
    DifferentialMatcher(const uint64_t _max_num_index_table_entries,
                        const uint64_t _max_num_target_table_entries,
                        // How many data/addresses to track per index/target PC
                        const uint64_t _max_num_tracked_items_per_table_entry);
    ~DifferentialMatcher() = default;

    bool isFull() const;

    bool hasCandidate(const Addr index_pc, const Addr target_pc) const;

    // Add a new candidate pair for matching.
    bool addCandidate(const Addr index_pc, const Addr target_pc);

    // We need to track the data from the index PC accesses, and the effective
    // addresses accessed by the target PC.
    // From the paper's Figure 10, we track the cache hit and cache fill events
    // from the index PC, and all cache events from the target PC (note that,
    // a cache miss would induce a cache fill later, so we only need to track
    // cache hit and cache misses for the target PC).
    void trackCacheHit(
      const Addr pc, const Addr effective_vaddr, const uint64_t data
    );
    void trackCacheMiss(const Addr pc, const Addr effective_vaddr);
    void trackCacheFill(
      const Addr pc, const Addr effective_vaddr, const uint64_t data
    );

  private:
    // Data structure to hold candidate pairs.
    // For each candidate pair, we have two tracking entries, one for index PC
    // and one for target PC.
    using IndexPcTrackingEntry = TrackingEntry;
    using TargetPcTrackingEntry = TrackingEntry;
    using CandidatePcPair = std::pair<Addr, Addr>; // <index_pc, target_pc>
    using TrackingPair = std::pair<
      IndexPcTrackingEntry, TargetPcTrackingEntry
    >;
    std::map<CandidatePcPair, TrackingPair> candidate_index_target_pc;
};

} // namespace prefetch
} // namespace gem5

#endif // __DMP_DIFFERENTIAL_MATCHER_HH__
