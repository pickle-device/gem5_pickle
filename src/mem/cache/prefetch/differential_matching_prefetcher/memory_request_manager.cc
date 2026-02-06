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

#include "mem/cache/prefetch/differential_matching_prefetcher/memory_request_manager.hh"

#include "arch/generic/mmu.hh"
#include "base/logging.hh"
#include "mem/cache/prefetch/differential_matching_prefetcher/prefetch_queue.hh"
#include "mem/request.hh"
#include "sim/clock_domain.hh"
#include "sim/eventq.hh"

namespace gem5
{

namespace prefetch
{

namespace dmp
{

MemoryRequestBookkeeper::MemoryRequestBookkeeper(
    const Addr _request_vaddr, const Addr _request_paddr,
    const uint64_t _request_size, const RequestorID _requestor_id,
    const Addr _pc, const Tick _earliest_issue_tick,
    const bool has_physical_address
) : request_vaddr(_request_vaddr), request_paddr(_request_paddr),
    request_size(_request_size), requestor_id(_requestor_id), pc(_pc),
    earliest_issue_tick(_earliest_issue_tick), request(nullptr),
    has_physical_address(has_physical_address)
{
}

MemoryRequestBookkeeper*
MemoryRequestBookkeeper::createPrefetchRequestUsingVirtualAddr(
    const Addr _request_vaddr, const uint64_t _request_size,
    const RequestorID _requestor_id, const Addr _pc,
    const Tick _earliest_issue_tick
)
{
    // We don't have the physical address in this case, so we use a dummy
    // value, signifying that the physical address is not known yet.
    return new MemoryRequestBookkeeper(
        /*request_vaddr*/ _request_vaddr,
        /*request_paddr*/ 0xBADC0DE,
        /*request_size*/ _request_size,
        /*requestor_id*/ _requestor_id,
        /*pc*/ _pc,
        /*earliest_issue_tick*/ _earliest_issue_tick,
        /*has_physical_address*/ false
    );
}

MemoryRequestBookkeeper*
MemoryRequestBookkeeper::createPrefetchRequestUsingPhysicalAddr(
    const Addr _request_paddr, const uint64_t _request_size,
    const RequestorID _requestor_id, const Addr _pc,
    const Tick _earliest_issue_tick
)
{
    // We don't have the virtual address in this case, so we use the physical
    // address for both virtual and physical addresses.
    return new MemoryRequestBookkeeper(
        /*request_vaddr*/ _request_paddr,
        /*request_paddr*/ _request_paddr,
        /*request_size*/ _request_size,
        /*requestor_id*/ _requestor_id,
        /*pc*/ _pc,
        /*earliest_issue_tick*/ _earliest_issue_tick,
        /*has_physical_address*/ true
    );
}

RequestPtr
MemoryRequestBookkeeper::getRequest()
{
    if (request == nullptr) {
        // Create the request if it doesn't exist yet
        request = std::make_shared<Request>(
            /* vaddr */ request_vaddr,
            /* size */ request_size,
            // We set the prefetch flag so that the request can be treated as a
            // prefetch request by the cache and memory system
            /* flags */ Request::PREFETCH,
            /* id */ requestor_id,
            /* pc */ pc,
            /* context id */ 0,
            /* atomic op */ nullptr
        );
        if (has_physical_address) {
            request->setPaddr(request_paddr);
        }
    }
    return request;
}

bool
MemoryRequestBookkeeper::hasPhysicalAddress() const
{
    return has_physical_address;
}

MemoryRequestManager::MemoryRequestManager(
    PrefetchQueue* _owner, ClockDomain* _clock_domain,
    uint64_t _cache_block_size, const RequestorID _requestor_id, BaseMMU* _mmu,
    const Cycles _request_propagation_delay
) : owner(_owner), clock_domain(_clock_domain),
    cache_block_size(_cache_block_size),
    requestor_id(_requestor_id), mmu(_mmu),
    request_propagation_delay_in_cycles(_request_propagation_delay),
    skip_address_translation(_mmu != nullptr),
    processPendingTranslationQueueEvent(
        [this]{ processPendingTranslationQueue(); },
        "DMP MemoryRequestManager Process Pending Translation Queue Event"
    ),
    processPendingMemoryQueueEvent(
        [this]{ processPendingMemoryQueue(); },
        "DMP MemoryRequestManager Process Pending Memory Queue Event"
    ),
    processCompletedRequestEvent(
        [this]{ processCompletedRequestQueue(); },
        "DMP MemoryRequestManager Process Completed Request Queue Event"
    )
{
}

bool
MemoryRequestManager::enqueuePrefetchRequestUsingVirtualAddr(
    Addr block_aligned_vaddr, Addr pc
)
{
    panic_if(
        skip_address_translation,
        "MemoryRequestManager is configured to skip address translation, "
        "cannot enqueue prefetch request using virtual address."
    );
    DMP_MEMORY_MANAGER_DEBUG(
        "Enqueue prefetch request using vaddr 0x%llx, "
        "earliest issue tick %lld\n", block_aligned_vaddr,
        curTick() + clock_domain->cyclesToTicks(
            request_propagation_delay_in_cycles
        )
    );

    // Check if there is already an outstanding request for this address
    if (outstanding_requests.find(block_aligned_vaddr) !=
        outstanding_requests.end()) {
        DMP_MEMORY_MANAGER_DEBUG(
            "There is already an outstanding request for vaddr 0x%llx, "
            "not enqueuing a new request.\n",
            block_aligned_vaddr
        );
        return false;
    }

    // Create a bookkeeper for this prefetch request
    MemoryRequestBookkeeper* bookkeeper =
        MemoryRequestBookkeeper::createPrefetchRequestUsingVirtualAddr(
            /*_request_vaddr*/ block_aligned_vaddr,
            /*_request_size*/ cache_block_size,
            /*_requestor_id*/ requestor_id,
            /*_pc*/ pc,
            /*_earliest_issue_tick*/ curTick() + clock_domain->cyclesToTicks(
                request_propagation_delay_in_cycles
            )
        );
    outstanding_requests[block_aligned_vaddr] = bookkeeper;
    pending_translation_queue.push(bookkeeper);

    scheduleSendAddressTranslationRequestsEvent();
    return true;
}

bool
MemoryRequestManager::enqueuePrefetchRequestUsingPhysicalAddr(
    Addr block_aligned_paddr, Addr pc
)
{
    DMP_MEMORY_MANAGER_DEBUG(
        "Enqueue prefetch request using paddr 0x%llx, "
        "earliest issue tick %lld\n",
        block_aligned_paddr,
        curTick() + clock_domain->cyclesToTicks(
            request_propagation_delay_in_cycles
        )
    );
    // Check if there is already an outstanding request for this address
    if (outstanding_requests.find(block_aligned_paddr) !=
        outstanding_requests.end()) {
        DMP_MEMORY_MANAGER_DEBUG(
            "There is already an outstanding request for paddr 0x%llx, "
            "not enqueuing a new request.\n",
            block_aligned_paddr
        );
        return false;
    }

    // Create a bookkeeper for this prefetch request
    MemoryRequestBookkeeper* bookkeeper =
        MemoryRequestBookkeeper::createPrefetchRequestUsingPhysicalAddr(
            /*_request_vaddr*/ block_aligned_paddr,
            /*_request_size*/ cache_block_size,
            /*_requestor_id*/ requestor_id,
            /*_pc*/ pc,
            /*_earliest_issue_tick*/ curTick() + clock_domain->cyclesToTicks(
                request_propagation_delay_in_cycles
            )
        );
    outstanding_requests[block_aligned_paddr] = bookkeeper;
    pending_memory_queue.push(bookkeeper);

    scheduleSendMemoryRequestsEvent();
    return true;
}

void
MemoryRequestManager::processPendingTranslationQueue()
{

}

void
MemoryRequestManager::processPendingMemoryQueue()
{

}

void
MemoryRequestManager::processCompletedRequestQueue()
{

}

void
MemoryRequestManager::scheduleSendAddressTranslationRequestsEvent()
{
    const bool event_already_scheduled =
        processPendingTranslationQueueEvent.scheduled();
    const bool has_pending_translation = !pending_translation_queue.empty();
    if (!event_already_scheduled && has_pending_translation) {
        const Tick scheduled_tick =
            std::max(
                pending_translation_queue.front()->earliest_issue_tick,
                curTick() + clock_domain->cyclesToTicks(Cycles(1))
            );
        owner->schedule(processPendingTranslationQueueEvent, scheduled_tick);
    }
}

void
MemoryRequestManager::scheduleSendMemoryRequestsEvent()
{
    const bool event_already_scheduled =
        processPendingMemoryQueueEvent.scheduled();
    const bool has_pending_memory_request = !pending_memory_queue.empty();
    if (!event_already_scheduled && has_pending_memory_request) {
        const Tick scheduled_tick =
            std::max(
                pending_translation_queue.front()->earliest_issue_tick,
                curTick() + clock_domain->cyclesToTicks(Cycles(1))
            );
        owner->schedule(processPendingMemoryQueueEvent, scheduled_tick);
    }
}

void
MemoryRequestManager::scheduleProcessCompletedRequestQueueEvent()
{
    const bool event_already_scheduled =
        processCompletedRequestEvent.scheduled();
    const bool has_completed_request = !completed_request_queue.empty();
    if (!event_already_scheduled && has_completed_request) {
        const Tick scheduled_tick =
            std::max(
                pending_translation_queue.front()->earliest_issue_tick,
                curTick() + clock_domain->cyclesToTicks(Cycles(1))
            );
        owner->schedule(processCompletedRequestEvent, scheduled_tick);
    }
}

} // namespace dmp

} // namespace prefetch

} // namespace gem5
