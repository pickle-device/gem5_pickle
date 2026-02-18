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

#include "mem/cache/prefetch/differential_matching_prefetcher/prefetch_queue.hh"

#include <deque>
#include <list>
#include <tuple>
#include <unordered_set>
#include <utility>

#include "base/statistics.hh"
#include "base/stats/group.hh"
#include "base/types.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "debug/DifferentialMatchingPrefetcherPrefetchQueueDebug.hh"
#include "debug/DifferentialMatchingPrefetcherPrefetchQueueStuckDebug.hh"
#include "enums/CacheLevel.hh"
#include "mem/cache/prefetch/differential_matching_prefetcher/differential_matching_prefetcher_interface.hh"
#include "mem/cache/prefetch/differential_matching_prefetcher/prefetch_request.hh"
#include "mem/cache/prefetch/differential_matching_prefetcher/util.hh"
#include "mem/packet.hh"
#include "mem/ruby/common/DataBlock.hh"
#include "mem/ruby/protocol/CHI/Cache_CacheEntry.hh"
#include "mem/ruby/protocol/CHI/Cache_Controller.hh"
#include "params/DifferentialMatchingPrefetcherPrefetchQueue.hh"

namespace gem5
{

namespace prefetch
{

namespace dmp
{

PrefetchQueue::PrefetchQueue(
    const DifferentialMatchingPrefetcherPrefetchQueueParams& params
)
  : ClockedObject(params),
    system(params.system),
    cache_controller(nullptr),
    cache_controller_level(params.cache_level),
    owner(nullptr),
    queue_size(params.queue_size),
    cache_block_size(params.system->cacheLineSize()),
    block_shift(log2(params.system->cacheLineSize())),
    process_pending_new_prefetch_requests_event(
        [this]{ processPendingNewPrefetchRequests(); },
        "Prefetch Queue Process Pending New Prefetch Requests Event"
    ),
    send_prefetched_data_from_stride_prefetcher_to_dmp_event(
        [this]{ processPendingStridePrefetchResults(); },
        "Prefetch Queue Process Pending Stride Prefetch Results Event"
    ),
    local_cache_data_access_delay(params.local_cache_data_access_delay),
    request_propagation_delay(params.request_propagation_delay),
    skip_address_translation(params.mmu == nullptr),
    memory_request_manager(
        /*owner*/ this,
        /*clock_domain*/ params.clock_domain,
        /*cache_block_size*/ cache_block_size,
        /*requestor_id*/ params.system->getRequestorId(this),
        /*thread_context*/ (params.associated_cpu == nullptr)
            ? nullptr
            : params.associated_cpu->getContext(0),
        /*mmu*/ params.mmu,
        /*local_cache_data_access_delay*/ local_cache_data_access_delay,
        /*request_propagation_delay*/ request_propagation_delay
    ),
    indirect_relation_table(nullptr),
    recentlyIssuedPrefetches(/*capacity*/ 64),
    stats(this)
{
}

void
PrefetchQueue::regStats()
{
    ClockedObject::regStats();
}


void
PrefetchQueue::preDumpStats()
{
    statistics::Group::preDumpStats();

    DMP_PREFETCH_QUEUE_STUCK_DEBUG(
        "Predump stats for PrefetchQueue %s\n", name()
    );
    DMP_PREFETCH_QUEUE_STUCK_DEBUG(
        "Current prefetch queue size: %lu\n", getQueueSize()
    );
    DMP_PREFETCH_QUEUE_STUCK_DEBUG("Current tick: %lu\n", curTick());

    // Now that we are about to exit the simulation, we want to know if there
    // are prefetches that got stuck in the queue and never got fulfilled.
    const Tick cur_tick = curTick();
    for (const auto& entry : prefetch_requests) {
        const std::list<PrefetchRequest>& requests = entry.second;
        for (const PrefetchRequest& request : requests) {
            const Tick request_latency =
                cur_tick - request.getQueueEnteringTick();
            if (request_latency > cyclesToTicks(Cycles(10000))) {
                stats.num_prefetch_requests_stuck++;
                stats.prefetch_request_stuck_duration_histogram.sample(
                    request_latency
                );
            }
        }
    }
}

void
PrefetchQueue::setOwner(DifferentialMatchingPrefetcherInterface* dmp)
{
    owner = dmp;
}

void
PrefetchQueue::setCacheController(ruby::AbstractController* _cache_controller)
{
    cache_controller = dynamic_cast<ruby::CHI::Cache_Controller*>(
        _cache_controller
    );
    assert(cache_controller != nullptr);
    memory_request_manager.setCacheController(cache_controller);
}

void
PrefetchQueue::setIndirectRelationTable(IndirectRelationTable* irt)
{
    indirect_relation_table = irt;
}

bool
PrefetchQueue::enqueuePendingRequest(PrefetchRequest prefetch_request)
{
    bool can_coalesce = false;
    const Addr prefetch_vaddr_block_aligned =
        (prefetch_request.prefetch_vaddr >> block_shift) << block_shift;

    auto prefetch_request_it = prefetch_requests.find(
        prefetch_vaddr_block_aligned
    );
    const Addr prefetch_pc = prefetch_request.target_pc;
    can_coalesce = (prefetch_request_it != prefetch_requests.end());

    stats.num_enqueued_requests++;
    if (can_coalesce) {
        stats.num_requests_after_coalescing++;
    }

    if (can_coalesce) {
        // Coalesce the prefetch request
        std::list<PrefetchRequest> &existing_request =
            prefetch_request_it->second;
        DMP_PREFETCH_QUEUE_DEBUG(
            "Coalescing prefetch request for vaddr 0x%llx\n",
            prefetch_request.prefetch_vaddr
        );
        prefetch_request.profileQueueEnteringTick();
        existing_request.push_back(std::move(prefetch_request));
    } else {
        // Enqueue the new prefetch request
        // TODO: implement a more sophisticated replacement policy when the
        // queue is full, instead of simply rejecting
        if (isFull()) {
            stats.num_dropped_requests_due_to_full_queue++;
            DMP_PREFETCH_QUEUE_DEBUG(
                "Prefetch queue is full. Cannot enqueue prefetch "
                "request for vaddr 0x%llx\n",
                prefetch_request.prefetch_vaddr
            );
            prefetch_request.is_dropped = true;
            prefetch_request.profileQueueLeavingTick();
            stats.prefetch_queue_occupancy_histogram.sample(getQueueSize());
            return false;
        }
        DMP_PREFETCH_QUEUE_DEBUG(
            "Enqueuing new prefetch request for vaddr 0x%llx\n",
            prefetch_request.prefetch_vaddr
        );
        prefetch_request.profileQueueEnteringTick();
        prefetch_requests[prefetch_vaddr_block_aligned].emplace_back(
            prefetch_request
        );
        owner->notifyNewPrefetchRequest(cache_controller_level);
        if (skip_address_translation) {
            memory_request_manager.enqueuePrefetchRequestUsingPhysicalAddr(
                prefetch_vaddr_block_aligned, prefetch_pc
            );
        } else {
            memory_request_manager.enqueuePrefetchRequestUsingVirtualAddr(
                prefetch_vaddr_block_aligned, prefetch_pc
            );
        }
    }
    stats.prefetch_queue_occupancy_histogram.sample(getQueueSize());
    return true;
}

uint64_t
PrefetchQueue::getQueueSize() const
{
    const uint64_t current_size =
        prefetch_requests.size() + countNumPendingNewRequestsAfterCoalescing();
    return current_size;
}

bool
PrefetchQueue::isFull() const
{
    const uint64_t current_size = getQueueSize();
    return current_size >= queue_size;
}

void
PrefetchQueue::processPendingNewPrefetchRequests()
{
    while (!pending_new_requests.empty()) {
        PrefetchRequest prefetch_request = pending_new_requests.front();
        pending_new_requests.pop_front();
        enqueuePendingRequest(prefetch_request);
    }
}

void
PrefetchQueue::processPendingStridePrefetchResults()
{
    while (!pending_stride_prefetch_results.empty()) {
        std::tuple<Addr, Addr, uint64_t> stride_prefetch_result =
            pending_stride_prefetch_results.front();
        const Addr stride_prefetch_vaddr = std::get<0>(stride_prefetch_result);
        const Addr stride_pc = std::get<1>(stride_prefetch_result);
        const uint64_t prefetched_data = std::get<2>(stride_prefetch_result);
        pending_stride_prefetch_results.pop();
        // Process the stride prefetch result
        owner->handleNewPrefetchedDataFromStridePrefetcher(
            /*target_paddr*/ stride_prefetch_vaddr,
            /*pc*/ stride_pc,
            /*data*/ prefetched_data
        );
    }
}

void
PrefetchQueue::scheduleProcessPendingNewPrefetchRequestsEvent()
{
    const bool event_already_scheduled =
        process_pending_new_prefetch_requests_event.scheduled();
    const bool has_pending_new_requests = !pending_new_requests.empty();
    if (!event_already_scheduled && has_pending_new_requests) {
        const Tick scheduled_tick = curTick() + cyclesToTicks(Cycles(1));
        schedule(process_pending_new_prefetch_requests_event, scheduled_tick);
    }
}

void
PrefetchQueue::scheduleProcessPendingStridePrefetchResultsEvent()
{
    const bool event_already_scheduled =
        send_prefetched_data_from_stride_prefetcher_to_dmp_event.scheduled();
    const bool has_pending_stride_prefetch_results =
        !pending_stride_prefetch_results.empty();
    if (!event_already_scheduled && has_pending_stride_prefetch_results) {
        const Tick scheduled_tick = curTick() + cyclesToTicks(Cycles(1));
        schedule(
            send_prefetched_data_from_stride_prefetcher_to_dmp_event,
            scheduled_tick
        );
    }
}

bool
PrefetchQueue::hasPendingMemoryRequests() const
{
    return memory_request_manager.hasPendingMemoryRequests();
}

Tick
PrefetchQueue::getNextReadyRequestTick() const
{
    return memory_request_manager.getNextReadyRequestTick();
}

PacketPtr
PrefetchQueue::getNextRequestPacket()
{
    return memory_request_manager.getNextRequestPacket();
}

void
PrefetchQueue::trackCacheHit(PacketPtr pkt)
{
    const Addr paddr = pkt->req->getPaddr();
    const Addr block_aligned_paddr = paddr & ~(cache_block_size - 1);
    DMP_PREFETCH_QUEUE_DEBUG(
        "Tracking cache hit for paddr 0x%llx (size %lu)\n",
        paddr, pkt->getSize()
    );
    notifyMemoryRequestCompleted(pkt);
}

void
PrefetchQueue::trackCacheMiss(PacketPtr pkt)
{
    // We don't need to do anything upon cache miss, as we will receive a
    // memory response when the memory request is completed, and we will
    // process the completed prefetch request and generate new prefetch
    // requests at that time.
}

void
PrefetchQueue::trackCacheFill(PacketPtr pkt)
{
    const Addr paddr = pkt->req->getPaddr();
    const Addr block_aligned_paddr = paddr & ~(cache_block_size - 1);
    DMP_PREFETCH_QUEUE_DEBUG(
        "Tracking cache fill for paddr 0x%llx (size %lu)\n",
        paddr, pkt->getSize()
    );
    notifyMemoryRequestCompleted(pkt);
}

void
PrefetchQueue::notifyMemoryRequestCompleted(PacketPtr pkt)
{
    memory_request_manager.processMemoryResponse(pkt);
}

void
PrefetchQueue::processCompletedPrefetchRequest(
    const Addr prefetch_vaddr_block_aligned,
    const std::vector<uint8_t>& response_data,
    const bool is_prefetch_hit_in_local_cache,
    const bool translation_fault
)
{
    auto prefetch_request_it = prefetch_requests.find(
        prefetch_vaddr_block_aligned
    );
    if (prefetch_request_it == prefetch_requests.end()) {
        DMP_PREFETCH_QUEUE_DEBUG(
            "Received completed prefetch request for vaddr block 0x%llx, "
            "but no outstanding prefetch request found for this address.\n",
            prefetch_vaddr_block_aligned
        );
        return;
    }

    std::list<PrefetchRequest> &requests = prefetch_request_it->second;
    for (PrefetchRequest &prefetch_request : requests) {
        if (translation_fault) {
            stats.num_dropped_requests_due_to_translation_fault++;
            DMP_PREFETCH_QUEUE_DEBUG(
                "Translation fault for prefetch request with vaddr 0x%llx, "
                "pc 0x%llx. Dropping this prefetch request.\n",
                prefetch_request.prefetch_vaddr, prefetch_request.target_pc
            );
        } else if (is_prefetch_hit_in_local_cache) {
            stats.num_requests_fulfilled_by_local_cache++;
        } else {
            stats.num_requests_fulfilled_by_prefetching++;
        }
        prefetch_request.profileQueueLeavingTick();

        // We only want to track prefetch request latency for the requests that
        // are fulfilled, so we don't track the latency for the requests that
        // are dropped due to translation fault.
        if (translation_fault) {
            continue;
        }
        stats.prefetch_request_latency_histogram.sample(
            prefetch_request.getPrefetchLatency()
        );
        const Addr request_vaddr = prefetch_request.prefetch_vaddr;
        const Addr target_pc = prefetch_request.target_pc;
        DMP_PREFETCH_QUEUE_DEBUG(
            "Processing completed prefetch request for vaddr 0x%llx, "
            "pc 0x%llx\n",
            request_vaddr, target_pc
        );
        if (!prefetch_request.setResponseFromCacheBlockData(
            /*cache_block_data*/ response_data.data(),
            /*cache_block_size*/ cache_block_size
        )) {
            DMP_PREFETCH_QUEUE_DEBUG(
                "Failed to set response for prefetch request with vaddr "
                "0x%llx\n, data_size %lu, cache_block_size %lu\n",
                request_vaddr, prefetch_request.size, cache_block_size
            );
            continue;
        }

        // If the cache level is L1, then this is the prefetch queue of the
        // stride prefetcher, and we need to notify the DMP of the prefetched
        // data from the stride prefetcher.
        if (cache_controller_level == enums::CacheLevel::L1) {
            DMP_PREFETCH_QUEUE_DEBUG(
                "Notifying DMP of new prefetched data from stride prefetcher: "
                "prefetch_vaddr_block_aligned=0x%llx, target_paddr=0x%llx, "
                "target_pc=0x%llx, size=%lu, data=%#llx\n",
                prefetch_vaddr_block_aligned, request_vaddr, target_pc,
                prefetch_request.size, prefetch_request.getResponse()
            );
            pending_stride_prefetch_results.push(
                std::make_tuple(
                    request_vaddr, target_pc, prefetch_request.getResponse()
                )
            );
            scheduleProcessPendingStridePrefetchResultsEvent();
            continue;
        }

        // Now we consult the IRT to generate new prefetch requests based on
        // the matching results. If there's no IRT, this is the stride
        // prefetcher.
        if (indirect_relation_table == nullptr) {
            // If the IRT is not set, it's a prefetcher that does not generate
            // new prefetch requests based on the IRT, so we can skip the IRT
            // query and directly continue to the next request.
            continue;
        }
        std::optional<std::vector<PrefetchRequest>> new_prefetch_requests =
            indirect_relation_table->queryEntryByIndexPc(
                /*index_pc*/ target_pc, // the target_pc now becomes the
                                        // index_pc for the IRT query
                /*data_from_index_pc*/ prefetch_request.getResponse()
            );
        if (new_prefetch_requests.has_value()) {
            for (const PrefetchRequest &new_request :
                new_prefetch_requests.value()) {
                const Addr new_prefetch_vaddr = new_request.prefetch_vaddr;
                if (recentlyIssuedPrefetches.contains(new_prefetch_vaddr)) {
                    PrefetcherStats& stats = owner->getStats();
                    stats.numDMPPrefetchesDroppedDueToRepeatedPrefetches++;
                    DMP_PREFETCH_QUEUE_DEBUG(
                        "Dropping new prefetch request generated from IRT for "
                        "index pc 0x%llx, target pc 0x%llx, vaddr 0x%llx, "
                        "because a prefetch request for the same address has "
                        "been issued recently.\n",
                        target_pc, new_request.target_pc,
                        new_request.prefetch_vaddr
                    );
                    continue;
                }
                recentlyIssuedPrefetches.push(new_prefetch_vaddr);
                pending_new_requests.push_back(new_request);
                DMP_PREFETCH_QUEUE_DEBUG(
                    "Generated new prefetch request from IRT for index pc "
                    "0x%llx, target pc 0x%llx, vaddr 0x%llx\n",
                    target_pc, new_request.target_pc,
                    new_request.prefetch_vaddr
                );
                scheduleProcessPendingNewPrefetchRequestsEvent();
                owner->getStats().numDMPPrefetchesEmitted++;
            }
        } else {
            DMP_PREFETCH_QUEUE_DEBUG(
                "No matching entry found in IRT for index pc 0x%llx, "
                "data from index pc %lu\n",
                target_pc, prefetch_request.getResponse()
            );
        }
    }

    prefetch_requests.erase(prefetch_request_it);
    stats.prefetch_queue_occupancy_histogram.sample(getQueueSize());
    // Remove the completed prefetch request from the queue
    DMP_PREFETCH_QUEUE_DEBUG(
        "Removing completed prefetch request for vaddr block 0x%llx from "
        "prefetch queue, queue size %lu\n",
        prefetch_vaddr_block_aligned, prefetch_requests.size()
    );
}

void
PrefetchQueue::recheckPendingPrefetchRequests()
{
    owner->notifyNewPrefetchRequest(cache_controller_level);
}

uint64_t
PrefetchQueue::countNumPendingNewRequestsAfterCoalescing() const
{
    std::unordered_set<Addr> unique_block_aligned_addresses;
    for (const auto& entry : pending_new_requests) {
        unique_block_aligned_addresses.insert(
            entry.prefetch_vaddr >> block_shift
        );
    }
    return unique_block_aligned_addresses.size();
}

PrefetchQueue::PrefetchQueueStats::PrefetchQueueStats(
    statistics::Group* parent
) : statistics::Group(parent, "PrefetchQueueStats"),
    ADD_STAT(
        num_enqueued_requests, statistics::units::Count::get(),
        "Number of prefetch requests enqueued to the prefetch queue"
    ),
    ADD_STAT(
        num_requests_after_coalescing, statistics::units::Count::get(),
        "Number of prefetch requests after coalescing the requests for the "
        "same cache block"
    ),
    ADD_STAT(
        num_dropped_requests_due_to_full_queue,
        statistics::units::Count::get(),
        "Number of prefetch requests dropped due to full prefetch queue"
    ),
    ADD_STAT(
        num_dropped_requests_due_to_translation_fault,
        statistics::units::Count::get(),
        "Number of prefetch requests dropped due to translation fault"
    ),
    ADD_STAT(
        num_requests_fulfilled_by_local_cache, statistics::units::Count::get(),
        "Number of prefetch requests fulfilled by local cache hits"
    ),
    ADD_STAT(
        num_requests_fulfilled_by_prefetching, statistics::units::Count::get(),
        "Number of prefetch requests fulfilled by prefetching"
    ),
    ADD_STAT(
        prefetch_queue_occupancy_histogram, statistics::units::Count::get(),
        "Histogram of the occupancy of the prefetch queue when a new request "
        "is enqueued"
    ),
    ADD_STAT(
        prefetch_request_latency_histogram, statistics::units::Tick::get(),
        "Histogram of the latency of prefetch requests from enqueueing to "
        "being fulfilled"
    ),
    ADD_STAT(
        num_prefetch_requests_stuck, statistics::units::Count::get(),
        "Number of prefetch requests that got stuck in the prefetch queue "
        "for more than 10000 cycles and never got fulfilled"
    ),
    ADD_STAT(
        prefetch_request_stuck_duration_histogram,
        statistics::units::Tick::get(),
        "Histogram of the duration that prefetch requests got stuck in the "
        "prefetch queue without being fulfilled"
    )
{
}

void
PrefetchQueue::PrefetchQueueStats::regStats()
{
    statistics::Group::regStats();
    prefetch_queue_occupancy_histogram
        .init(16)
        .flags(statistics::pdf);
    prefetch_request_latency_histogram
        .init(16)
        .flags(statistics::pdf);
    prefetch_request_stuck_duration_histogram
        .init(16)
        .flags(statistics::pdf);
}

}; // namespace dmp

}; // namespace prefetch

}; // namespace gem5
