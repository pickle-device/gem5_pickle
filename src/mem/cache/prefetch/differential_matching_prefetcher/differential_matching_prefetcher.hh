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
#include "base/trace.hh"
#include "base/types.hh"
#include "mem/cache/prefetch/differential_matching_prefetcher/differential_matching_prefetcher_interface.hh"
#include "mem/cache/prefetch/differential_matching_prefetcher/index_queue.hh"
#include "mem/cache/prefetch/differential_matching_prefetcher/indirection_candidate_scoreboard.hh"
#include "mem/cache/prefetch/differential_matching_prefetcher/stride_tracker.hh"
#include "mem/cache/simple_cache_probe_arg.hh"
#include "mem/packet.hh"
#include "mem/ruby/slicc_interface/AbstractController.hh"
#include "params/DifferentialMatchingPrefetcher.hh"
#include "sim/eventq.hh"
#include "sim/probe/probe.hh"
#include "sim/system.hh"

#define DMP_PREFETCHER_DEBUG(...) \
    DPRINTF(DifferentialMatchingPrefetcherDebug, \
            "(DMP) " __VA_ARGS__)

namespace gem5
{

namespace prefetch
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
    ruby::AbstractController *l1_controller;
    EventFunctionWrapper process_detection_event;

  // prefetcher parameters
  private:
    const uint64_t index_queue_size;
    const uint64_t indirection_candidate_scoreboard_num_entries;
    const uint64_t indirection_candidate_scoreboard_num_candidates_per_entry;
    const uint64_t sample_window_size;

  // prefetcher components
  private:
    StrideTracker stride_tracker;
    IndexQueue index_queue;
    IndirectionCandidateScoreboard indirection_candidate_scoreboard;

  public:
    PARAMS(DifferentialMatchingPrefetcher);
    DifferentialMatchingPrefetcher(
      const DifferentialMatchingPrefetcherParams &p
    );
    ~DifferentialMatchingPrefetcher() override = default;
    void regProbeListeners() override;

  private:
    // What to do when an event happens?
    void processDetectionEvent();
    void scheduleHandleDetectionEvent();
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
    // Here, we receive a new candidate pair of PCs from the Indirection
    // Candidate Scoreboard (ICS). We can start differential matching for this
    // pair of PCs.
    void handleNewCandidateFromIcs(
      const Addr index_pc, const Addr target_pc
    ) override;
  // Helpers
    Addr getBlockAddress(Addr addr) const;
};

} // namespace prefetch
} // namespace gem5

#endif  //__DMP_HH__
