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
#include "mem/cache/prefetch/differential_matching_prefetcher/prefetch_request.hh"
#include "mem/packet.hh"
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
    indirect_relation_table(nullptr)
{
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
PrefetchQueue::trackL2CacheHit(PacketPtr pkt)
{
    const Addr paddr = pkt->req->getPaddr();
    if (prefetch_requests.find(paddr) != prefetch_requests.end()) {
        notifyMemoryRequestCompleted(pkt);
    }
}

void
PrefetchQueue::trackL2CacheMiss(PacketPtr pkt)
{
    const Addr paddr = pkt->req->getPaddr();
    const Addr pc = pkt->req->getPC();
    // We don't need to do anything upon cache miss, as we will receive a
    // memory response when the memory request is completed, and we will
    // process the completed prefetch request and generate new prefetch
    // requests at that time.

    // TODO: Remove this
    // Test Prefetching
    static uint64_t count = 0;
    if (indirect_relation_table->containsIndexPc(0x120) && pc == 0x120) {
        std::vector<PrefetchRequest> new_requests = {
            PrefetchRequest(
                /*target_pc*/ 0x140,
                /*prefetch_vaddr*/ 0x20000000 + count * 8,
                /*size*/ 8,
                /*irt_id*/ 0
            )
        };
        count++;
        for (const PrefetchRequest &new_request : new_requests) {
            enqueuePendingRequest(new_request);
        }
        DMP_PREFETCH_QUEUE_DEBUG(
            "Test: Enqueued new prefetch request for vaddr 0x%llx based on "
            "index pc 0x120\n",
            0x20000000 + (count - 1) * 8
        );
    }
}

void
PrefetchQueue::trackL2CacheFill(PacketPtr pkt)
{
    const Addr paddr = pkt->req->getPaddr();
    if (prefetch_requests.find(paddr) != prefetch_requests.end()) {
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
        const Addr request_vaddr = prefetch_request.prefetch_vaddr;
        const Addr target_pc = prefetch_request.target_pc;
        DMP_PREFETCH_QUEUE_DEBUG(
            "Processing completed prefetch request for vaddr 0x%llx, "
            "pc 0x%llx\n",
            request_vaddr, target_pc
        );
        if (!prefetch_request.setResponseFromCacheBlockData(
            response_data.data(), cache_block_size
        )) {
            DMP_PREFETCH_QUEUE_DEBUG(
                "Failed to set response for prefetch request with vaddr "
                "0x%llx\n, data_size %lu, cache_block_size %lu\n",
                request_vaddr, prefetch_request.size, cache_block_size
            );
            continue;
        }

        // Now we consult the IRT to generate new prefetch requests based on
        // the matching results.
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
            }
        } else {
            DMP_PREFETCH_QUEUE_DEBUG(
                "No matching entry found in IRT for index pc 0x%llx, "
                "data from index pc %lu\n",
                target_pc, prefetch_request.getResponse()
            );
        }
    }

    // Remove the completed prefetch request from the queue
    prefetch_requests.erase(prefetch_request_it);
}

}; // namespace dmp

}; // namespace prefetch

}; // namespace gem5
