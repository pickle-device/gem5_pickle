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

#ifndef __PREFETCH_AGENT_HH__
#define __PREFETCH_AGENT_HH__

#include "debug/DifferentialMatchingPrefetcherPrefetchAgentDebug.hh"
#include "mem/cache/prefetch/base.hh"
#include "mem/cache/prefetch/differential_matching_prefetcher/prefetch_queue.hh"
#include "params/PrefetchAgent.hh"

#define DMP_PREFETCH_AGENT_DEBUG(...) \
    DPRINTF(DifferentialMatchingPrefetcherPrefetchAgentDebug, \
            "(Prefetch Agent) " __VA_ARGS__)

namespace gem5
{

namespace prefetch
{

namespace dmp
{

// We don't want to use prefetch::Queued as we want to have more control over
// prefetch scheduling.
class PrefetchAgent : public prefetch::Base
{
  private:
    // The prefetch queue that this agent will use to manage its prefetch
    // requests.
    PrefetchQueue* prefetch_queue;
  public:
    PARAMS(PrefetchAgent);
    PrefetchAgent(const PrefetchAgentParams& params);

    // The prefetcher is notified of cache accesses (hits/misses) through this
    // function.
    void notify(
      const CacheAccessProbeArg &acc, const PrefetchInfo &pfi
    ) override;

    // The prefetcher is notified of cache fills through this function.
    void notifyFill(const CacheAccessProbeArg &acc) override;

    // The prefetcher is notified of cache evictions through this function.
    void notifyEvict(const EvictionInfo &info) override;

    // This function is called by the prefetcher proxy to obtain the next
    // prefetch request to be issued.
    PacketPtr getPacket() override;

    // This function is called by the prefetcher proxy to determine when the
    // next prefetch request will be ready to be issued.
    Tick nextPrefetchReadyTime() const override;
};  // class PrefetchAgent

}; // namespace dmp

}; // namespace prefetch

}; // namespace gem5

#endif // __PREFETCH_AGENT_HH__
