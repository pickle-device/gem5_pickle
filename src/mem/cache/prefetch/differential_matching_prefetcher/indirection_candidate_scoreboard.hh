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
#include <utility>
#include <vector>

#include "base/logging.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "debug/DifferentialMatchingPrefetcherIndirectionCandidateScoreboardDebug.hh"
#include "mem/cache/prefetch/differential_matching_prefetcher/differential_matching_prefetcher_interface.hh"

#define DMP_ICS_DEBUG(...) \
    DPRINTF(DifferentialMatchingPrefetcherIndirectionCandidateScoreboardDebug,\
            "(ICS) " __VA_ARGS__)

namespace gem5
{
namespace prefetch
{

class CandidateEntry
{
  public:
    Addr pc;
    uint64_t l1_cache_miss_count;
    CandidateEntry(
        const Addr _pc, const uint64_t _l1_cache_miss_count
    );
    void profileL1CacheMiss();
}; // class CandidateEntry

class IndirectionCandidateScoreboardEntry
{
  private:
    uint64_t max_num_candidates; // Maximum number of candidates to track
    uint64_t sample_window_size;  // Number of cache misses to track
    Addr index_pc;
    std::vector<CandidateEntry> candidates;
    uint64_t tracked_l1_cache_miss_count;
  public:
    IndirectionCandidateScoreboardEntry(
        const Addr index_pc, const uint64_t _capacity,
        const uint64_t _sample_window_size
    );
    Addr getIndexPC() const;
    void trackL1CacheMiss(const Addr target_pc);
    // Return the candidate PCs with their L1 cache miss counts
    std::vector<std::pair<Addr, uint64_t>> \
      getCandidatesWithL1CacheMissCount() const;
    bool isSampleWindowFull() const;
}; // class IndirectionCandidateScoreboardEntry

struct PcPairHash
{
    std::size_t operator () (const std::pair<Addr, Addr> &p) const {
        return p.first ^ p.second;
    }
};

class IndirectionCandidateScoreboard
{
  public:
    // Maximum number of entries in the scoreboard
    const uint64_t max_num_entries;
    // Maximum number of candidates per entry
    const uint64_t max_num_candidates;
    // Number of cache misses to track per entry
    const uint64_t sample_window_size;
    // This is my patch preventing previously unsuccessful matches from being
    // prioritized when adding new candidates
    const bool deprioritize_previously_unsuccessful_match;
    std::unordered_map<std::pair<Addr, Addr>, uint64_t, PcPairHash>
        previously_unsuccessful_matches;
    std::vector<IndirectionCandidateScoreboardEntry> scoreboard;
    DifferentialMatchingPrefetcherInterface *prefetcher_interface;

    IndirectionCandidateScoreboard(
      const uint64_t _max_num_entries, const uint64_t _max_num_candidates,
      const uint64_t _sample_window_size,
      bool _deprioritize_previously_unsuccessful_match,
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
    void trackL1CacheMiss(const Addr target_pc);
    // Handle the update of previously unsuccessful matches
    void markPreviouslyUnsuccessfulMatch(
        const Addr index_pc, const Addr target_pc
    );
    // Get the candidate target PC with the highest L1 cache miss count
    Addr getTargetPcWithHighestL1CacheMissCount(
        const Addr index_pc
    ) const;
    // Get the candidate target PC with the deprioritized scoring scheme,
    // i.e., previously unsuccessful matches are deprioritized by scaling down
    // their scores by the number of times they were unsuccessful.
    Addr getTargetPcWithHighestL1ScoreAfterDeprioritization(
        const Addr index_pc
    ) const;
    // Weighting scheme: we want a function that decays slowly at first, then
    // decays faster as the number of unsuccessful attempts increases. Though,
    // we want the scoring scheme to be simple so if the number of unsuccessful
    // attempts is very high, we pretty much give up on the candidate.
    // Here's what I came up with:
    //     adjusted_score = original_score * w
    // where w = ((1-1/(9-x))-0.4375)*2.2 if (x<8) else 1/(2*x)
    // and x is the number of times the candidate was previously unsuccessful.
    // x | w
    // --+----------------
    // 1 0.9625000000000001
    // 2 0.9232142857142859
    // 3 0.8708333333333335
    // 4 0.7975000000000002
    // 5 0.6875
    // 6 0.5041666666666669
    // 7 0.1375
    // 8 0.0625
    // 9 0.05555555555555555
    // 10 0.05
    // 11 0.045454545454545456
    // 12 0.041666666666666664
    // 13 0.038461538461538464
    // 14 0.03571428571428571
    // 15 0.03333333333333333
    // 16 0.03125
    // 17 0.029411764705882353
    // 18 0.027777777777777776
    // 19 0.02631578947368421
    // 20 0.025
    // Why?
    //   - We want w to be in the range of (0, 1] so that the adjusted score
    //     is always less than or equal to the original score.
    //   - Initially, we pick the giving up threshold to be 8 unsuccessful
    //     attempts. This means that if a candidate was previously unsuccessful
    //     8 times, its score is scaled down to 6.25% of the original score.
    //   - For the first 7 unsuccessful attempts,
    //       + Initially, we pick w = 1-1/(9-x), which decays faster as x
    //         increases. However, w ranges is [0.5, 0.875]. While at x = 8,
    //         w drops to 1/2*8=0.0625, which is a big drop. Also, at x = 0,
    //         w = 1, which also causes a big jump from x = 0 to x = 1.
    //       + We rescale w to be in the range of [0.1375, 0.9625] to smoothen
    //         the transition (0.1375 is picked because it's double of 0.0625,
    //         the value at x = 8; and 0.9625 is picked to keep the drop from
    //         x = 0 to x = 1 smaller than x = 1 to x = 2).
    //       + This rescaling is done by:
    //           w = (w - (0.5 - 0.0625)) * (0.9625 - 0.1375) / (0.875 - 0.5)
    double getWeightedScore(uint64_t num_unsuccessful_attempts) const;
}; // class IndirectionCandidateScoreboard

} // namespace gem5
} // namespace prefetch

#endif // __DMP_INDIRECTION_CANDIDATE_SCOREBOARD_HH__
