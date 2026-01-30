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

#include "mem/cache/prefetch/differential_matching_prefetcher/indirection_candidate_scoreboard.hh"

#include <algorithm>
#include <cstdint>
#include <vector>

#include "base/logging.hh"
#include "base/types.hh"
#include "debug/DifferentialMatchingPrefetcherIndirectionCandidateScoreboardDebug.hh"

namespace gem5
{
namespace prefetch
{

CandidateEntry::CandidateEntry(
    const Addr _pc, const uint64_t _l1_cache_miss_count
) : pc(_pc), l1_cache_miss_count(_l1_cache_miss_count)
{
}

void
CandidateEntry::profileL1CacheMiss()
{
    l1_cache_miss_count++;
}

IndirectionCandidateScoreboardEntry::IndirectionCandidateScoreboardEntry(
  const Addr _index_pc, const uint64_t _capacity,
  const uint64_t _sample_window_size
) : max_num_candidates(_capacity), sample_window_size(_sample_window_size),
    index_pc(_index_pc), candidates(), tracked_l1_cache_miss_count(0)
{
    candidates.reserve(max_num_candidates);
}

Addr
IndirectionCandidateScoreboardEntry::getIndexPC() const
{
    return index_pc;
}

void
IndirectionCandidateScoreboardEntry::trackL1CacheMiss(const Addr target_pc)
{
    // if target_pc is the same as the index PC, ignore
    if (target_pc == index_pc) {
        return;
    }

    tracked_l1_cache_miss_count++;

    // Check if the PC is already in the candidates list
    for (auto &candidate : candidates) {
        if (candidate.pc == target_pc) {
            candidate.profileL1CacheMiss();
            return;
        }
    }

    // If not found, add a new candidate entry
    if (candidates.size() < max_num_candidates) {
        candidates.emplace_back(target_pc, 1);
    } else {
        // If full, ignore the new candidate
    }
}

Addr
IndirectionCandidateScoreboardEntry::\
    getCandidateTargetPcWithHighestL1CacheMissCount() const
{
    // Find the candidate with the highest L1 cache miss count
    auto best_candidate_it = candidates.begin();
    for (auto it = candidates.begin(); it != candidates.end(); ++it) {
        if (it->l1_cache_miss_count > best_candidate_it->l1_cache_miss_count) {
            best_candidate_it = it;
        }
    }

    // Record the candidate PC to return
    Addr candidate_pc = best_candidate_it->pc;

    return candidate_pc;
}

bool
IndirectionCandidateScoreboardEntry::isSampleWindowFull() const
{
    return tracked_l1_cache_miss_count >= sample_window_size;
}

IndirectionCandidateScoreboard::IndirectionCandidateScoreboard(
  const uint64_t _max_num_entries, const uint64_t _max_num_candidates,
  const uint64_t _sample_window_size,
  DifferentialMatchingPrefetcherInterface *_prefetcher_interface
) : max_num_entries(_max_num_entries),
    max_num_candidates(_max_num_candidates),
    sample_window_size(_sample_window_size),
    scoreboard(),
    prefetcher_interface(_prefetcher_interface)
{
    scoreboard.reserve(max_num_entries);
}

bool
IndirectionCandidateScoreboard::addEntry(const Addr index_pc)
{
    // If the index PC already exists, do nothing
    for (const auto &entry : scoreboard) {
        if (entry.getIndexPC() == index_pc) {
            return false;
        }
    }

    // If the scoreboard is not full, add a new entry
    if (scoreboard.size() < max_num_entries) {
        scoreboard.emplace_back(
            index_pc, max_num_entries, sample_window_size
        );
        DMP_ICS_DEBUG("Added index PC %#x to ICS\n", index_pc);
        return true;
    }
    return false;
}

bool
IndirectionCandidateScoreboard::containsEntry(const Addr index_pc) const
{
    for (const auto &entry :scoreboard) {
        if (entry.getIndexPC() == index_pc) {
            return true;
        }
    }
    return false;
}

void
IndirectionCandidateScoreboard::trackL1CacheMiss(const Addr target_pc)
{
    if (scoreboard.empty()) {
        return;
    }

    std::vector<Addr> pcs_to_remove;
    for (auto &entry : scoreboard) {
        entry.trackL1CacheMiss(target_pc);
        // what to do when sample window is full?
        // - first, we can notify the prefetcher of the new candidate
        // - then, we can remove the entry from the scoreboard
        if (entry.isSampleWindowFull()) {
            const Addr candidate_index_pc = entry.getIndexPC();
            const Addr candidate_target_pc =
                entry.getCandidateTargetPcWithHighestL1CacheMissCount();
            // Notify the prefetcher of the new candidate
            prefetcher_interface->handleNewCandidateFromIcs(
                candidate_index_pc, candidate_target_pc
            );
            // Mark the entry for removal
            pcs_to_remove.push_back(entry.getIndexPC());
        }
    }

    if (pcs_to_remove.empty()) {
        return;
    }

    // Remove entries whose sample window is full
    scoreboard.erase(
        std::remove_if(
            scoreboard.begin(), scoreboard.end(),
            [&pcs_to_remove](IndirectionCandidateScoreboardEntry &entry) {
                return std::find(
                    pcs_to_remove.begin(), pcs_to_remove.end(),
                    entry.getIndexPC()
                ) != pcs_to_remove.end();
            }
        ),
        scoreboard.end()
    );

    // ughh, sanity check the AI generated code
    // TODO: remove this.
    for (const auto &entry: scoreboard) {
        if (entry.isSampleWindowFull()) {
            panic("Entry with index PC %#x still has full sample window after "
                  "processing cache misses.", entry.getIndexPC());
        }
    }
}

} // namespace prefetch
} // namespace gem5
