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

#ifndef __DMP_INDIRECTION_CANDIDATE_SCOREBOARD_HH__
#define __DMP_INDIRECTION_CANDIDATE_SCOREBOARD_HH__

#include <cstdint>
#include <vector>

#include "base/types.hh"
#include "mem/cache/prefetch/differential_matching_prefetcher/differential_matching_prefetcher_interface.hh"

namespace gem5
{
namespace prefetch
{

class CandidateEntry
{
  public:
    Addr pc;
    uint64_t cache_miss_count;
    CandidateEntry(
        const Addr _pc, const uint64_t _cache_miss_count
    );
    void profileCacheMiss();
}; // class CandidateEntry

class IndirectionCandidateScoreboardEntry
{
  private:
    uint64_t max_num_candidates; // Maximum number of candidates to track
    uint64_t sample_window_size;  // Number of cache misses to track
    Addr index_pc;
    std::vector<CandidateEntry> candidates;
    uint64_t tracked_cache_miss_count;
  public:
    IndirectionCandidateScoreboardEntry(
        const Addr index_pc, const uint64_t _capacity,
        const uint64_t _sample_window_size
    );
    Addr getIndexPC() const;
    void trackCacheMiss(const Addr target_pc);
    // Return the candidate PC with the highest cache miss count
    Addr getCandidateTargetPcWithHighestCacheMissCount() const;
    bool isSampleWindowFull() const;
}; // class IndirectionCandidateScoreboardEntry

class IndirectionCandidateScoreboard
{
  public:
    uint64_t max_num_entries; // Maximum number of entries in the scoreboard
    uint64_t max_num_candidates; // Maximum number of candidates per entry
    uint64_t sample_window_size; // Number of cache misses to track per entry
    std::vector<IndirectionCandidateScoreboardEntry> scoreboard;
    DifferentialMatchingPrefetcherInterface *prefetcher_interface;

    IndirectionCandidateScoreboard(
      const uint64_t _max_num_entries, const uint64_t _max_num_candidates,
      const uint64_t _sample_window_size,
      DifferentialMatchingPrefetcherInterface *_prefetcher_interface
    );
    // Add a new entry for the given index PC. Return true if added
    // successfully.
    bool addEntry(const Addr index_pc);
    // Check if the given index PC exists in the scoreboard.
    bool containsEntry(const Addr index_pc) const;
    // Notify the scoreboard of a cache miss.
    // If any entry's sample window is full, notify the prefetcher interface
    // of the new candidate and remove the entry from the scoreboard.
    void trackCacheMiss(const Addr target_pc);
}; // class IndirectionCandidateScoreboard

} // namespace gem5
} // namespace prefetch

#endif // __DMP_INDIRECTION_CANDIDATE_SCOREBOARD_HH__
