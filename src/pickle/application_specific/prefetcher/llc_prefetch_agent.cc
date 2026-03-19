/*
 * Copyright (c) 2025 The Regents of the University of California
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "pickle/application_specific/prefetcher/llc_prefetch_agent.hh"

#include <cassert>
#include <memory>
#include <utility>

#include "base/trace.hh"
#include "debug/LLCPrefetchAgentDebug.hh"
#include "mem/request.hh"

namespace gem5
{

LLCPrefetchAgent::LLCPrefetchAgent(const LLCPrefetchAgentParams &params)
    : ClockedObject(params),
      system(params.system),
      prefetcher(nullptr),
      llc_controller(params.llc_controller),
      addr_ranges(params.addr_ranges),
      requestor_id(system->getRequestorId(this)),
      ticks_per_cycle(250), // running at the LLC frequency
      processOutgoingRequestQueueEvent(
        [this]() { processOutgoingRequestQueue(); },
        name() + ".process_outgoing_request_queue_event"
    ),
      mem_side_port(name() + ".mem_side_port", this),
      agent_stats(this)
{
    assert(llc_controller != nullptr);
    DPRINTF(LLCPrefetchAgentDebug,
        "LLC Prefetch Agent created, monitoring address ranges:\n"
    );
    for (const auto& range : addr_ranges) {
        DPRINTF(LLCPrefetchAgentDebug, "  %s\n", range.to_string());
    }
}

LLCPrefetchAgent::~LLCPrefetchAgent()
{
}

void LLCPrefetchAgent::setPicklePrefetcher(PicklePrefetcher* prefetcher)
{
    assert(prefetcher != nullptr);
    this->prefetcher = prefetcher;
}

void
LLCPrefetchAgent::enqueueRequestWithPAddr(const PrefetchRequest& pf_request)
{
    assert(pf_request.hasPAddr());
    agent_stats.prefetch_request_count++;
    prefetch_request_queue.push(pf_request);
    agent_stats.prefetch_request_queue_length.sample(
        prefetch_request_queue.size()
    );
    // Add to the map of outstanding requests
    pf_paddr_to_outstanding_requests[
        pf_request.getPrefetchPAddr()
    ] = pf_request;
    // Schedule the processing event if not already scheduled
    if (!processOutgoingRequestQueueEvent.scheduled()) {
        schedule(
            processOutgoingRequestQueueEvent,
            //clockEdge(0)
            curTick() + ticks_per_cycle
        );
    }
    DPRINTF(LLCPrefetchAgentDebug,
        "Enqueued prefetch request for paddr 0x%llx\n",
        pf_request.getPrefetchPAddr()
    );
}

void
LLCPrefetchAgent::completeRequest(Addr paddr)
{
    auto it = pf_paddr_to_outstanding_requests.find(paddr);
    if (it != pf_paddr_to_outstanding_requests.end()) {
        PrefetchRequest pf_request = it->second;
        // Notify the prefetcher that this prefetch request is completed
        prefetcher->agentCompletePrefetchRequest(pf_request);
        // Remove from the map of outstanding requests
        pf_paddr_to_outstanding_requests.erase(it);
        DPRINTF(LLCPrefetchAgentDebug,
            "Completed prefetch request for paddr 0x%llx\n", paddr
        );
    } else {
        DPRINTF(LLCPrefetchAgentDebug,
            "Received completion for unknown paddr 0x%llx\n", paddr
        );
    }
}

bool
LLCPrefetchAgent::isAddressInMonitoredRanges(Addr addr) const
{
    for (const auto& range : addr_ranges) {
        if (range.contains(addr)) {
            return true;
        }
    }
    return false;
}

void
LLCPrefetchAgent::processOutgoingRequestQueue()
{
    // Try to send as many requests as possible in the queue
    while (!prefetch_request_queue.empty()) {
        // Peek at the front request
        const PrefetchRequest& pf_request = prefetch_request_queue.top();
        const Addr paddr = pf_request.getPrefetchPAddr();
        // Check if the cache line is already present in the cache by
        // consulting the LLC directory and its own cache.
        // Note that the LLC directory does not keep track of cache lines only
        // present in LLC.
        if (llc_controller->getDirEntry(paddr) != nullptr
            || llc_controller->getCacheEntry(paddr) != nullptr) {
            // Cache line is already present, drop the request
            agent_stats.prefetch_request_dropped_due_to_cache_line_presence++;
            prefetch_request_queue.pop();
            agent_stats.prefetch_request_queue_length.sample(
                prefetch_request_queue.size()
            );
            // Notify the prefetcher that this prefetch request is "completed"
            completeRequest(paddr);
            DPRINTF(LLCPrefetchAgentDebug,
                "Dropped prefetch request for paddr 0x%llx as it is "
                "already present in the cache\n", paddr
            );
            continue;
        }
        // Try to send it out
        PacketPtr pkt = createPrefetchPacket(pf_request);
        bool success = mem_side_port.sendTimingReq(pkt);
        // If sent, pop it from the queue
        // If not sent, stop processing further requests
        if (success) {
            prefetch_request_queue.pop();
            agent_stats.prefetch_request_sent++;
            agent_stats.prefetch_request_queue_length.sample(
                prefetch_request_queue.size()
            );
            DPRINTF(LLCPrefetchAgentDebug,
                "Sent prefetch request for paddr 0x%llx\n", paddr
            );
        } else {
            // Failed to send, will retry later
            delete pkt;
            DPRINTF(LLCPrefetchAgentDebug,
                "Failed to send prefetch request for paddr 0x%llx\n", paddr
            );
            break;
        }
    }

    // Schedule the event again if there are still requests in the queue
    if (!prefetch_request_queue.empty()) {
        if (!processOutgoingRequestQueueEvent.scheduled()) {
            schedule(
                processOutgoingRequestQueueEvent,
                curTick() + ticks_per_cycle
            );
        }
    }
}

void LLCPrefetchAgent::triggerTests()
{
    // Trigger some test prefetch requests for testing purposes
    // Here we just enqueue some prefetch requests to some hardcoded
    // physical addresses for testing
    std::vector <Addr> test_paddrs = {
        0x110000000,
        0x110000000 + 1 * 64,
        0x110000000 + 2 * 64,
        0x110000000 + 3 * 64,
        0x110000000 + 4 * 64,
        0x110000000 + 5 * 64,
        0x110000000 + 6 * 64,
        0x110000000 + 7 * 64,
        0x110000000 + 8 * 64,
        0x110000000 + 9 * 64,
    };
    for (const auto& paddr : test_paddrs) {
        if (isAddressInMonitoredRanges(paddr)) {
            PrefetchRequest pf_request = PrefetchRequest::createWithPAddr(
                paddr, 0x0, curTick(), (paddr - 0x110000000) / 64, true,
                MaxTick - curTick() // priority_score
            );
            enqueueRequestWithPAddr(std::move(pf_request));
            DPRINTF(LLCPrefetchAgentDebug,
                "Triggered test prefetch request for paddr 0x%llx\n", paddr
            );
        } else {
            //DPRINTF(LLCPrefetchAgentDebug,
            //    "Test prefetch request for paddr 0x%llx is out of "
            //    "monitored ranges, not enqueued\n", paddr
            //);
        }
    }
}

PacketPtr LLCPrefetchAgent::createPrefetchPacket(
    const PrefetchRequest& pf_request) const
{
    const uint64_t cache_line_size = system->cacheLineSize();
    assert(pf_request.hasPAddr());
    Addr paddr = pf_request.getPrefetchPAddr();
    // Create a read prefetch packet
    Request::Flags flags = 0;
    RequestPtr req = std::make_shared<Request>(
        paddr, // physical address
        cache_line_size, // size
        flags,
        requestor_id
    );
    PacketPtr pkt = Packet::createRead(req);
    // Allocate a data buffer for the packet
    pkt->dataDynamic(new uint8_t[cache_line_size]);
    return pkt;
}

LLCPrefetchAgent::LLCPrefetchAgentRequestPort::LLCPrefetchAgentRequestPort(
    const std::string& name, LLCPrefetchAgent* owner
) : RequestPort(name)
{
    this->owner = owner;
}

LLCPrefetchAgent::LLCPrefetchAgentRequestPort::~LLCPrefetchAgentRequestPort()
{
}

bool
LLCPrefetchAgent::LLCPrefetchAgentRequestPort::recvTimingResp(PacketPtr pkt)
{
    // Notify the prefetcher that this prefetch request is "completed"
    const Addr paddr = pkt->req->getPaddr();
    owner->completeRequest(paddr);
    DPRINTF(LLCPrefetchAgentDebug,
        "Received prefetch response for paddr 0x%llx\n", paddr
    );
    // Do nothing with the response packet as the prefetcher does not read data
    delete pkt;
    return true;
}

void LLCPrefetchAgent::LLCPrefetchAgentRequestPort::recvReqRetry()
{
    // Try to send the next request in the queue if any
    owner->processOutgoingRequestQueue();
}

Port& LLCPrefetchAgent::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "mem_side_port") {
        return mem_side_port;
    }
    return ClockedObject::getPort(if_name, idx);
}
LLCPrefetchAgent::LLCPrefetchAgentStats::LLCPrefetchAgentStats(
    statistics::Group *parent)
    : statistics::Group(parent, "llc_prefetch_agent"),
      ADD_STAT(prefetch_request_count, statistics::units::Count::get(),
               "Number of prefetch requests received by the prefetcher"),
      ADD_STAT(prefetch_request_dropped_due_to_cache_line_presence,
               statistics::units::Count::get(),
               "Number of prefetch requests dropped due to the cache line "
               "already being present in the cache"),
      ADD_STAT(prefetch_request_sent, statistics::units::Count::get(),
               "Number of prefetch requests sent to the memory system"),
      ADD_STAT(prefetch_request_not_sent, statistics::units::Count::get(),
               "Number of prefetch requests not sent to the memory system "
               "due to our errors. Should be 0.",
               prefetch_request_count - prefetch_request_sent - \
                prefetch_request_dropped_due_to_cache_line_presence),
      ADD_STAT(prefetch_request_queue_length, statistics::units::Count::get(),
                "Histogram of the prefetch request queue length over time")
{
}

void LLCPrefetchAgent::LLCPrefetchAgentStats::regStats()
{
    statistics::Group::regStats();
    prefetch_request_queue_length
        .init(16)
        .flags(statistics::pdf);
}

}; // namespace gem5
