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

#include <list>

#include "base/types.hh"
#include "enums/CacheLevel.hh"
#include "mem/cache/prefetch/differential_matching_prefetcher/differential_matching_prefetcher_interface.hh"
#include "mem/cache/prefetch/differential_matching_prefetcher/prefetch_request.hh"
#include "mem/cache/prefetch/differential_matching_prefetcher/util.hh"
#include "mem/packet.hh"
#include "mem/ruby/common/DataBlock.hh"
#include "mem/ruby/protocol/CHI/Cache_CacheEntry.hh"
#include "mem/ruby/protocol/CHI/Cache_Controller.hh"
#include "params/PrefetchQueue.hh"

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
    request_propagation_delay(params.request_propagation_delay),
    skip_address_translation(params.mmu == nullptr),
    memory_request_manager(
        /*owner*/ this,
        /*clock_domain*/ params.clock_domain,
        /*cache_block_size*/ cache_block_size,
        /*requestor_id*/ params.system->getRequestorId(this),
        /*mmu*/ params.mmu,
        /*request_propagation_delay*/ request_propagation_delay
    ),
    indirect_relation_table(nullptr),
    stats(this)
{
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

    // Check if the data is already in the cache. Since the protocol does
    // not record a local prefetch hit as a hit event, we need to directly
    // acquire the data from the controller.
    ruby::CHI::Cache_CacheEntry* entry = cache_controller->getCacheEntry(
        prefetch_vaddr_block_aligned
    );
    if (entry != nullptr) {
        stats.num_requests_fulfilled_by_local_cache++;
        DMP_PREFETCH_QUEUE_DEBUG(
            "Prefetch request for vaddr 0x%llx hits in cache. "
            "No need to enqueue the request.\n",
            prefetch_request.prefetch_vaddr
        );
        // We can directly process the completed prefetch request without
        // sending a memory request, as the data is already in the cache.
        // TODO: model the delay of sending the request to L1/L2 and getting
        // the response back
        const ruby::DataBlock& response_data = entry->getDataBlk();
        const uint8_t* response_data_ptr = response_data.getData(
            0, cache_block_size
        );
        std::vector<uint8_t> response_data_vec(
            response_data_ptr, response_data_ptr + cache_block_size
        );
        processCompletedPrefetchRequest(
            prefetch_vaddr_block_aligned, response_data_vec
        );
        return true;
    }

    if (can_coalesce) {
        // Coalesce the prefetch request
        std::list<PrefetchRequest> &existing_request =
            prefetch_request_it->second;
        DMP_PREFETCH_QUEUE_DEBUG(
            "Coalescing prefetch request for vaddr 0x%llx\n",
            prefetch_request.prefetch_vaddr
        );
        existing_request.push_back(std::move(prefetch_request));
    } else {
        // Enqueue the new prefetch request
        // TODO: implement a more sophisticated replacement policy when the
        // queue is full, instead of simply rejecting
        if (isFull()) {
            DMP_PREFETCH_QUEUE_DEBUG(
                "Prefetch queue is full. Cannot enqueue prefetch "
                "request for vaddr 0x%llx\n",
                prefetch_request.prefetch_vaddr
            );
            return false;
        }
        DMP_PREFETCH_QUEUE_DEBUG(
            "Enqueuing new prefetch request for vaddr 0x%llx\n",
            prefetch_request.prefetch_vaddr
        );
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
    return true;
}

bool
PrefetchQueue::isFull() const
{
    const uint64_t current_size = prefetch_requests.size();
    return current_size >= queue_size;
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
    if (
        prefetch_requests.find(block_aligned_paddr) != prefetch_requests.end()
    ) {
        notifyMemoryRequestCompleted(pkt);
    }
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
    if (
        prefetch_requests.find(block_aligned_paddr) != prefetch_requests.end()
    ) {
        notifyMemoryRequestCompleted(pkt);
    }
}

void
PrefetchQueue::notifyMemoryRequestCompleted(PacketPtr pkt)
{
    memory_request_manager.processMemoryResponse(pkt);
}

void
PrefetchQueue::processCompletedPrefetchRequest(
    const Addr prefetch_vaddr_block_aligned,
    const std::vector<uint8_t>& response_data
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
        stats.num_requests_fulfilled_by_prefetching++;
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
            owner->handleNewPrefetchedDataFromStridePrefetcher(
                /*target_paddr*/ request_vaddr,
                /*pc*/ target_pc,
                /*data*/ prefetch_request.getResponse()
            );
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
                enqueuePendingRequest(new_request);
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
    // Remove the completed prefetch request from the queue
    DMP_PREFETCH_QUEUE_DEBUG(
        "Removing completed prefetch request for vaddr block 0x%llx from "
        "prefetch queue, queue size %lu\n",
        prefetch_vaddr_block_aligned, prefetch_requests.size()
    );
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
        num_requests_fulfilled_by_local_cache, statistics::units::Count::get(),
        "Number of prefetch requests fulfilled by local cache hits"
    ),
    ADD_STAT(
        num_requests_fulfilled_by_prefetching, statistics::units::Count::get(),
        "Number of prefetch requests fulfilled by prefetching"
    )
{
}

}; // namespace dmp

}; // namespace prefetch

}; // namespace gem5
