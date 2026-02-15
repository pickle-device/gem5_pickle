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

#ifndef __DMP_HH__
#define __DMP_HH__

#include <string>
#include <unordered_map>
#include <vector>

#include "base/cache/associative_cache.hh"
#include "base/sat_counter.hh"
#include "base/statistics.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "debug/DifferentialMatchingPrefetcherCacheObserverDebug.hh"
#include "debug/DifferentialMatchingPrefetcherDebug.hh"
#include "mem/cache/prefetch/differential_matching_prefetcher/differential_matcher.hh"
#include "mem/cache/prefetch/differential_matching_prefetcher/differential_matching_prefetcher_interface.hh"
#include "mem/cache/prefetch/differential_matching_prefetcher/index_queue.hh"
#include "mem/cache/prefetch/differential_matching_prefetcher/indirect_relation_table.hh"
#include "mem/cache/prefetch/differential_matching_prefetcher/indirection_candidate_scoreboard.hh"
#include "mem/cache/prefetch/differential_matching_prefetcher/prefetch_queue.hh"
#include "mem/cache/prefetch/differential_matching_prefetcher/stride_tracker.hh"
#include "mem/cache/simple_cache_probe_arg.hh"
#include "mem/packet.hh"
#include "mem/ruby/slicc_interface/AbstractController.hh"
#include "params/DifferentialMatchingPrefetcher.hh"
#include "sim/clock_domain.hh"
#include "sim/eventq.hh"
#include "sim/probe/probe.hh"
#include "sim/system.hh"

#define DMP_PREFETCHER_DEBUG(...) \
    DPRINTF(DifferentialMatchingPrefetcherDebug, \
            "(DMP) " __VA_ARGS__)

#define DMP_CACHE_OBSERVER_DEBUG(...) \
    DPRINTF(DifferentialMatchingPrefetcherCacheObserverDebug, \
            "(Cache Observer) " __VA_ARGS__)

namespace gem5
{

namespace prefetch
{

namespace dmp
{

/**
 * Implementation of the Differential-Matching Prefetcher (DMP).
 *
 * References:
 * Differential-Matching Prefetcher for Indirect Memory Access.
 * Fu, G., Xia, T., Luo, Z., Chen, R., Zhao, W., & Ren, P. (2024, March).
 * Differential-matching prefetcher for indirect memory access.
 * In 2024 IEEE International Symposium on High-Performance Computer
 * Architecture (HPCA) (pp. 439-453). IEEE.
 */

class DifferentialMatchingPrefetcher : \
  public ProbeListenerObject, public DifferentialMatchingPrefetcherInterface
{
  // simulation components
  private:
    System *system;
    const uint64_t cache_line_size;
    ClockDomain *clock_domain;
    const Addr memory_size_in_bytes;
    PrefetchQueue *dmp_prefetch_queue;
    PrefetchQueue *stride_prefetch_queue;
    ruby::AbstractController *l1_controller;
    ruby::AbstractController *l2_controller;
    const bool enable_dmp_prefetching;

  // prefetcher parameters
  private:
    const uint64_t index_queue_size;
    const uint64_t indirection_candidate_scoreboard_num_entries;
    const uint64_t indirection_candidate_scoreboard_num_candidates_per_entry;
    const uint64_t sample_window_size;
    const bool ics_deprioritize_on_unsuccessful_matching_patch;

  // prefetcher components
  private:
    StrideTracker stride_tracker;
    IndexQueue index_queue;
    IndirectionCandidateScoreboard indirection_candidate_scoreboard;
    DifferentialMatcher differential_matcher;
    IndirectRelationTable indirect_relation_table;

  public:
    PARAMS(DifferentialMatchingPrefetcher);
    DifferentialMatchingPrefetcher(
      const DifferentialMatchingPrefetcherParams &p
    );
    ~DifferentialMatchingPrefetcher() override = default;
    void regProbeListeners() override;
    void regStats() override;

    std::string getPrefetcherName() const override;

  private:
    // Helper functions for processing events
    void promoteIndexPcFromIqToIcs();
    void addIndirectionCandidateToDifferentialMatcher(
      const Addr index_pc, const Addr target_pc
    );

  // L1 cache data access observers
  private:
    // Determine whether the prefetcher should observe this access.
    bool isObservable(const SimpleCacheAccessProbeArg &arg);
    // Observing an L1 hit
    void observeL1CacheHit (const SimpleCacheAccessProbeArg &arg);
    // Observing an L1 miss
    void observeL1CacheMiss(const SimpleCacheAccessProbeArg &arg);
    // Observing an L1 fill (writeback)
    void observeL1CacheFill(const SimpleCacheAccessProbeArg &arg);
  // Events from prefetcher components
  private:
    // Here, we receive a new stride detection from the Stride Tracker.
    // We inject the PC into the Index Queue.
    void handleNewlyDetectedStride(const Addr pc) override;
    void handleIcsHasAvailableSlots() override;
    // Here, we receive a new candidate pair of PCs from the Indirection
    // Candidate Scoreboard (ICS). We can start differential matching for this
    // pair of PCs.
    void handleNewCandidateFromIcs(
      const Addr index_pc, const Addr target_pc
    ) override;
    // Here, we receive a differential matching result for a candidate pair of
    // PCs from the Differential Matcher.
    void handleDifferentialMatchResult(
      const Addr index_pc, const Addr target_pc, const bool successful_match,
      const Addr target_base_vaddr, const int64_t shift_amount,
      const AccessType index_access_type, const AccessType target_access_type
    ) override;
    // Forward the notification from the prefetch queue to the prefetch proxy
    // to trigger the scheduling of prefetch requests when there is a new
    // prefetch request to be scheduled from prefetch queue.
    void notifyNewPrefetchRequest(
      const enums::CacheLevel cache_controller_level
    ) override;
    // Here, we receive the prefetched data from the stride prefetcher. We use
    // this data to find out if it is part of the IRT table. If it is, we
    // use this data to generate new prefetch requests for the next level
    // of indirection.
    void handleNewPrefetchedDataFromStridePrefetcher(
      const Addr target_paddr, const Addr pc, const uint64_t data
    ) override;
    // Allow other prefetchers to query whether DMP already prefetches for
    // a specific PC.
    bool isATargetPC(const Addr pc) const override;

    // Helpers
    Addr getBlockAddress(Addr addr) const;
    uint64_t getDataFromProbe(const SimpleCacheAccessProbeArg &arg) const;

  public:
    PrefetcherStats stats;
    PrefetcherStats& getStats() override;

};

} // namespace dmp

} // namespace prefetch

} // namespace gem5

#endif  //__DMP_HH__
