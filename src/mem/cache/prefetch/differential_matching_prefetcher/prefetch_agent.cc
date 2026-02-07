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

#include "mem/cache/prefetch/differential_matching_prefetcher/prefetch_agent.hh"

namespace gem5
{

namespace prefetch
{

namespace dmp
{

PrefetchAgent::PrefetchAgent(const PrefetchAgentParams& params)
  : Base(params),
    prefetch_queue(params.prefetch_queue)
{
}

void
PrefetchAgent::notify(const CacheAccessProbeArg &acc, const PrefetchInfo &pfi)
{
    const Addr vaddr = pfi.getAddr();
    const Addr paddr = acc.pkt->req->getPaddr();
    const Addr pc = acc.pkt->req->getPC();
    const bool is_miss = pfi.isCacheMiss();
    const bool has_data = pfi.requestHasData();
    DMP_PREFETCH_AGENT_CACHE_OBSERVER_DEBUG(
        "Notify called with CacheAccessProbeArg: vaddr=0x%llx, paddr=0x%llx, "
        "pc=0x%llx, is_miss=%d, has_data=%d\n",
        vaddr, paddr, pc, is_miss, has_data
    );
    if (!is_miss){
        prefetch_queue->trackL2CacheHit(acc.pkt);
    } else {
        prefetch_queue->trackL2CacheMiss(acc.pkt);
    }
}

void
PrefetchAgent::notifyFill(const CacheAccessProbeArg &acc)
{
    const Addr paddr = acc.pkt->req->getPaddr();
    const Addr pc = acc.pkt->req->getPC();
    DMP_PREFETCH_AGENT_CACHE_OBSERVER_DEBUG(
        "NotifyFill called with CacheAccessProbeArg: paddr=0x%llx, "
        "pc=0x%llx\n",
        paddr, pc
    );
    prefetch_queue->trackL2CacheFill(acc.pkt);
}

void
PrefetchAgent::notifyEvict(const EvictionInfo &info)
{
    // I don't think we need to do anything upon cache eviction.
}

PacketPtr
PrefetchAgent::getPacket()
{
    PacketPtr pkt = prefetch_queue->getNextRequestPacket();
    DMP_PREFETCH_AGENT_DEBUG(
        "Sending packet for prefetch request with address 0x%llx\n",
        pkt ? pkt->req->getPaddr() : 0
    );
    return pkt;
}

Tick
PrefetchAgent::nextPrefetchReadyTime() const
{
    const Tick next_ready_tick = prefetch_queue->getNextReadyRequestTick();
    if (next_ready_tick != MaxTick) {
        DMP_PREFETCH_AGENT_DEBUG(
            "Next ready prefetch request will be ready at tick %lld\n",
            next_ready_tick
        );
    }
    return next_ready_tick;
}

}  // namespace dmp

}  // namespace prefetch

}  // namespace gem5
