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
#include <utility>
#include <vector>

#include "base/logging.hh"
#include "base/types.hh"
#include "debug/DifferentialMatchingPrefetcherIndirectionCandidateScoreboardDebug.hh"
#include "mem/cache/prefetch/differential_matching_prefetcher/differential_matching_prefetcher_interface.hh"

namespace gem5
{

namespace prefetch
{

namespace dmp
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
  const uint64_t _sample_window_size,
  DifferentialMatchingPrefetcherInterface* _prefetcher_interface
) : max_num_candidates(_capacity), sample_window_size(_sample_window_size),
    index_pc(_index_pc), candidates(), tracked_l1_cache_miss_count(0),
    prefetcher_interface(_prefetcher_interface)
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

std::vector<std::pair<Addr, uint64_t>>
IndirectionCandidateScoreboardEntry::getCandidatesWithL1CacheMissCount() const
{
    std::vector<std::pair<Addr, uint64_t>> candidate_miss_counts;
    for (const auto &candidate : candidates) {
        candidate_miss_counts.emplace_back(
            candidate.pc, candidate.l1_cache_miss_count
        );
    }
    return candidate_miss_counts;
}

bool
IndirectionCandidateScoreboardEntry::isSampleWindowFull() const
{
    return tracked_l1_cache_miss_count >= sample_window_size;
}

IndirectionCandidateScoreboard::IndirectionCandidateScoreboard(
  const uint64_t _max_num_entries, const uint64_t _max_num_candidates,
  const uint64_t _sample_window_size,
  const bool _deprioritize_previously_unsuccessful_match,
  DifferentialMatchingPrefetcherInterface *_prefetcher_interface
) : max_num_entries(_max_num_entries),
    max_num_candidates(_max_num_candidates),
    sample_window_size(_sample_window_size),
    deprioritize_previously_unsuccessful_match(
        _deprioritize_previously_unsuccessful_match
    ),
    prefetcher_interface(_prefetcher_interface)
{
    scoreboard.reserve(max_num_entries);
}

bool
IndirectionCandidateScoreboard::isFull() const
{
    return scoreboard.size() >= max_num_entries;
}

bool
IndirectionCandidateScoreboard::addEntry(const Addr index_pc)
{
    // If the index PC already exists, do nothing
    for (auto &entry : scoreboard) {
        if (entry.getIndexPC() == index_pc) {
            return false;
        }
    }

    // If the scoreboard is not full, add a new entry
    if (!isFull()) {
        scoreboard.emplace_back(
            index_pc, max_num_entries, sample_window_size, prefetcher_interface
        );
        DMP_ICS_DEBUG("Added index PC %#x to ICS\n", index_pc);
        // If still not full, notify the prefetcher interface that ICS has
        // available slots for new candidates
        if (!isFull()) {
            prefetcher_interface->handleIcsHasAvailableSlots();
        }
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
                deprioritize_previously_unsuccessful_match ?
                getTargetPcWithHighestL1ScoreAfterDeprioritization(
                    candidate_index_pc
                ) :
                getTargetPcWithHighestL1CacheMissCount(
                    candidate_index_pc
                );
            // Notify the prefetcher of the new candidate
            prefetcher_interface->handleNewCandidateFromIcs(
                candidate_index_pc, candidate_target_pc
            );
            // Mark the entry for removal
            pcs_to_remove.push_back(entry.getIndexPC());
            DMP_ICS_DEBUG(
                "Index PC %#x sample window full. New candidate to track: "
                "Index PC %#x, Target PC %#x\n",
                candidate_index_pc, candidate_index_pc, candidate_target_pc
            );
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
    prefetcher_interface->handleIcsHasAvailableSlots();
}

void
IndirectionCandidateScoreboard::markPreviouslySuccessfulMatch(
    const Addr index_pc, const Addr target_pc
)
{
    // Remove the entry from the previously unsuccessful matches map
    auto it = previously_unsuccessful_matches.find({index_pc, target_pc});
    if (it != previously_unsuccessful_matches.end()) {
        previously_unsuccessful_matches.erase(it);
    }
}

void
IndirectionCandidateScoreboard::markPreviouslyUnsuccessfulMatch(
    const Addr index_pc, const Addr target_pc
)
{
    previously_unsuccessful_matches[{index_pc, target_pc}]++;
}

Addr
IndirectionCandidateScoreboard::getTargetPcWithHighestL1CacheMissCount(
    const Addr index_pc
) const
{
    for (const auto &entry : scoreboard) {
        if (entry.getIndexPC() == index_pc) {
            Addr best_candidate_pc = 0;
            uint64_t highest_miss_count = 0;
            for (const auto &candidate_pair :
                 entry.getCandidatesWithL1CacheMissCount()) {
                if (candidate_pair.second > highest_miss_count) {
                    best_candidate_pc = candidate_pair.first;
                    highest_miss_count = candidate_pair.second;
                }
                DMP_ICS_DEBUG(
                    "Index PC %#x Candidate PC %#x L1 Cache Miss Count %lu\n",
                    index_pc, candidate_pair.first, candidate_pair.second
                );
            }
            return best_candidate_pc;
        }
    }
    return 0;
}

Addr
IndirectionCandidateScoreboard::\
    getTargetPcWithHighestL1ScoreAfterDeprioritization(
    const Addr index_pc
) const
{
    for (const auto &entry : scoreboard) {
        if (entry.getIndexPC() == index_pc) {
            Addr best_candidate_pc = 0;
            std::vector<std::pair<Addr, uint64_t>> candidate_scores =
                entry.getCandidatesWithL1CacheMissCount();
            // Deprioritize previously unsuccessful matches by scaling down
            // their scores by the number of times they were unsuccessful
            uint64_t highest_adjusted_score = 0;
            for (const auto &candidate_pair : candidate_scores) {
                Addr candidate_pc = candidate_pair.first;
                uint64_t original_score = candidate_pair.second;
                double adjusted_score = original_score;
                auto it =
                    previously_unsuccessful_matches.find(
                        {index_pc, candidate_pc}
                    );
                if (it != previously_unsuccessful_matches.end()) {
                    uint64_t unsuccess_count = it->second;
                    const double weight = getWeightedScore(unsuccess_count);
                    adjusted_score = \
                        static_cast<double>(original_score) * weight;
                }
                DMP_ICS_DEBUG(
                    "Index PC %#x Candidate PC %#x Original Score %lu "
                    "Adjusted Score %f\n",
                    index_pc, candidate_pc, original_score, adjusted_score
                );
                if (adjusted_score > highest_adjusted_score) {
                    highest_adjusted_score = adjusted_score;
                    best_candidate_pc = candidate_pc;
                }
            }
            return best_candidate_pc;
        }
    }
    return 0;
}

double
IndirectionCandidateScoreboard::getWeightedScore(
    const uint64_t num_unsuccessful_attempts
) const
{
    if (num_unsuccessful_attempts == 0) {
        return 1.0;
    } else if (num_unsuccessful_attempts < 8) {
        const double x = static_cast<double>(num_unsuccessful_attempts);
        return  ((1.0 - 1.0 / (9.0 - x)) - 0.4375) * 2.2;
    } else {
        return 1.0 / (2.0 * static_cast<double>(num_unsuccessful_attempts));
    }
}

} // namespace dmp

} // namespace prefetch

} // namespace gem5
