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
  : ProbeListenerObject(params),
    queue_size(params.queue_size),
    cache_block_size(params.system->cacheLineSize()),
    block_shift(log2(params.system->cacheLineSize())),
    request_propagation_delay(params.request_propagation_delay),
    skip_address_translation(params.mmu != nullptr),
    memory_request_manager(
        this, params.system->getRequestorId(this), params.mmu,
        request_propagation_delay
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
        prefetch_request.prefetch_vaddr >> block_shift;

    auto prefetch_request_it = prefetch_requests.find(
        prefetch_vaddr_block_aligned
    );
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
                prefetch_vaddr_block_aligned
            );
        } else {
            memory_request_manager.enqueuePrefetchRequestUsingVirtualAddr(
                prefetch_vaddr_block_aligned
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

}; // namespace dmp

}; // namespace prefetch

}; // namespace gem5
