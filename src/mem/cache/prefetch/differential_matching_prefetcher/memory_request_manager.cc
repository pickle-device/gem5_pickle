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

#include <cassert>
#include <cstdint>
#include <functional>
#include <utility>

#include "arch/generic/mmu.hh"
#include "base/logging.hh"
#include "base/statistics.hh"
#include "base/stats/group.hh"
#include "base/types.hh"
#include "debug/DifferentialMatchingPrefetcherMemoryRequestManagerDebug.hh"
#include "debug/DifferentialMatchingPrefetcherMemoryRequestManagerStuckDebug.hh"
#include "mem/cache/prefetch/differential_matching_prefetcher/prefetch_queue.hh"
#include "mem/cache/prefetch/differential_matching_prefetcher/util.hh"
#include "mem/packet.hh"
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
  AddressTranslationDoneCallbackType _translation_done_callback,
  AddressTranslationFaultCallbackType _translation_fault_callback,
  const Addr _request_vaddr, const Addr _request_paddr,
  const uint64_t _request_size, const RequestorID _requestor_id,
  const Addr _pc, const Tick _ready_tick, const bool has_physical_address
) : translation_done_callback(_translation_done_callback),
    translation_fault_callback(_translation_fault_callback),
    request_vaddr(_request_vaddr), request_paddr(_request_paddr),
    request_size(_request_size), requestor_id(_requestor_id), pc(_pc),
    local_cache_hit(false), ready_tick(_ready_tick),
    translation_fault(false),
    queue_entering_tick(0), translation_starting_tick(0),
    translation_finishing_tick(0), queue_leaving_tick(0),
    request(nullptr), packet(nullptr),
    has_physical_address(has_physical_address)
{
}

MemoryRequestBookkeeper::~MemoryRequestBookkeeper()
{
    // We don't need to manually delete the request as it is managed by a
    // shared pointer.
    // We also don't need to manually delete the packet as it will be deleted
    // by the prefetcher proxy.
}

MemoryRequestBookkeeper*
MemoryRequestBookkeeper::createPrefetchRequestUsingVirtualAddr(
    AddressTranslationDoneCallbackType _translation_done_callback,
    AddressTranslationFaultCallbackType _translation_fault_callback,
    const Addr _request_vaddr, const uint64_t _request_size,
    const RequestorID _requestor_id, const Addr _pc, const Tick _ready_tick
)
{
    // We don't have the physical address in this case, so we use a dummy
    // value, signifying that the physical address is not known yet.
    return new MemoryRequestBookkeeper(
        /*translation_done_callback*/ _translation_done_callback,
        /*translation_fault_callback*/ _translation_fault_callback,
        /*request_vaddr*/ _request_vaddr,
        /*request_paddr*/ 0xBADC0DE,
        /*request_size*/ _request_size,
        /*requestor_id*/ _requestor_id,
        /*pc*/ _pc,
        /*ready_tick*/ _ready_tick,
        /*has_physical_address*/ false
    );
}

MemoryRequestBookkeeper*
MemoryRequestBookkeeper::createPrefetchRequestUsingPhysicalAddr(
    AddressTranslationDoneCallbackType _translation_done_callback,
    AddressTranslationFaultCallbackType _translation_fault_callback,
    const Addr _request_paddr, const uint64_t _request_size,
    const RequestorID _requestor_id, const Addr _pc, const Tick _ready_tick
)
{
    // We don't have the virtual address in this case, so we use the physical
    // address for both virtual and physical addresses.
    return new MemoryRequestBookkeeper(
        /*translation_done_callback*/ _translation_done_callback,
        /*translation_fault_callback*/ _translation_fault_callback,
        /*request_vaddr*/ _request_paddr,
        /*request_paddr*/ _request_paddr,
        /*request_size*/ _request_size,
        /*requestor_id*/ _requestor_id,
        /*pc*/ _pc,
        /*ready_tick*/ _ready_tick,
        /*has_physical_address*/ true
    );
}

void
MemoryRequestBookkeeper::profileQueueEnteringTick()
{
    queue_entering_tick = curTick();
}

void
MemoryRequestBookkeeper::profileTranslationStartingTick()
{
    translation_starting_tick = curTick();
}

void
MemoryRequestBookkeeper::profileTranslationFinishingTick()
{
    translation_finishing_tick = curTick();
}

void
MemoryRequestBookkeeper::profileQueueLeavingTick()
{
    queue_leaving_tick = curTick();
}

Tick
MemoryRequestBookkeeper::getQueueEnteringTick() const
{
    return queue_entering_tick;
}

Tick
MemoryRequestBookkeeper::getQueueingDuration() const
{
    return queue_leaving_tick - queue_entering_tick;
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
        request->taskId(context_switch_task_id::Prefetcher);
        if (has_physical_address) {
            request->setPaddr(request_paddr);
        }
    }
    return request;
}

PacketPtr
MemoryRequestBookkeeper::getPacket()
{
    if (packet != nullptr) {
        return packet;
    }
    RequestPtr req = getRequest();
    packet = new Packet(req, MemCmd::HardPFReq);
    packet->allocate();
    return packet;
}

bool
MemoryRequestBookkeeper::hasPhysicalAddress() const
{
    return has_physical_address;
}

void
MemoryRequestBookkeeper::setDataFromPacket(PacketPtr pkt)
{
    const uint8_t* data_ptr = pkt->getConstPtr<uint8_t>();
    response_data.assign(data_ptr, data_ptr + request_size);
}

void
MemoryRequestBookkeeper::setDataFromDataBlock(
    const ruby::DataBlock& data_block, const uint64_t cache_block_size
)
{
    const uint8_t* data_ptr = data_block.getData(0, cache_block_size);
    response_data.assign(data_ptr, data_ptr + request_size);
}

void
MemoryRequestBookkeeper::setTranslationResult(
    const Fault &fault, const RequestPtr &req
)
{
    profileTranslationFinishingTick();
    if (fault) {
        translation_fault = true;
        translation_fault_callback(this, fault);
    } else {
        translation_fault = false;
        request_paddr = req->getPaddr();
        translation_done_callback(this);
    }
}

bool
MemoryRequestBookkeeper::isCompleted() const
{
    return !response_data.empty();
}

bool
MemoryRequestBookkeeper::isReady() const
{
    return curTick() >= ready_tick;
}

MemoryRequestManager::MemoryRequestManager(
    PrefetchQueue* _owner, ClockDomain* _clock_domain,
    uint64_t _cache_block_size, const RequestorID _requestor_id,
    ThreadContext* _thread_context, BaseMMU* _mmu,
    const Cycles _local_cache_data_access_delay_in_cycles,
    const Cycles _request_propagation_delay
) : owner(_owner),
    clock_domain(_clock_domain),
    cache_block_size(_cache_block_size),
    requestor_id(_requestor_id),
    thread_context(_thread_context),
    mmu(_mmu),
    cache_controller(nullptr),
    local_cache_data_access_delay_in_cycles(
        _local_cache_data_access_delay_in_cycles
    ),
    request_propagation_delay_in_cycles(_request_propagation_delay),
    skip_address_translation(_mmu == nullptr),
    previous_local_cache_access_completion_tick(0),
    completed_request_queue(0),
    process_pending_translation_queue_event(
        [this]{ processPendingTranslationQueue(); },
        "MemoryRequestManager Process Pending Translation Queue Event"
    ),
    process_completed_request_event(
        [this]{ processCompletedRequestQueue(); },
        "MemoryRequestManager Process Completed Request Queue Event"
    ),
    stats(owner, this, clock_domain)
{
    default_translation_done_callback = std::bind(
        &MemoryRequestManager::handleTranslationCompletion, this,
        std::placeholders::_1
    );
    default_translation_fault_callback = std::bind(
        &MemoryRequestManager::handleTranslationFault, this,
        std::placeholders::_1, std::placeholders::_2
    );
    panic_if_translation_done_callback = std::bind(
        &MemoryRequestManager::errorIfTranslationComplete, this,
        std::placeholders::_1
    );
    panic_if_translation_fault_callback = std::bind(
        &MemoryRequestManager::errorIfTranslationFault, this,
        std::placeholders::_1, std::placeholders::_2
    );
}

void
MemoryRequestManager::setCacheController(
    ruby::CHI::Cache_Controller* _cache_controller
)
{
    cache_controller = _cache_controller;
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
        "Enqueue prefetch request using vaddr 0x%llx, ready tick %lld\n",
        block_aligned_vaddr,
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
            /*translation_done_callback*/ default_translation_done_callback,
            /*translation_fault_callback*/ default_translation_fault_callback,
            /*_request_vaddr*/ block_aligned_vaddr,
            /*_request_size*/ cache_block_size,
            /*_requestor_id*/ requestor_id,
            /*_pc*/ pc,
            /*_ready_tick*/ curTick() + clock_domain->cyclesToTicks(
                request_propagation_delay_in_cycles
            )
        );
    outstanding_requests[block_aligned_vaddr] = bookkeeper;
    pending_translation_queue.push(bookkeeper);
    bookkeeper->profileQueueEnteringTick();
    stats.num_memory_request_enqueued++;

    scheduleSendAddressTranslationRequestsEvent();
    return true;
}

bool
MemoryRequestManager::enqueuePrefetchRequestUsingPhysicalAddr(
    Addr block_aligned_paddr, Addr pc
)
{
    const Addr block_aligned_vaddr = block_aligned_paddr;
    DMP_MEMORY_MANAGER_DEBUG(
        "Enqueue prefetch request using paddr 0x%llx, ready tick %lld\n",
        block_aligned_paddr,
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
        MemoryRequestBookkeeper::createPrefetchRequestUsingPhysicalAddr(
            /*translation_done_callback*/ panic_if_translation_done_callback,
            /*translation_fault_callback*/ panic_if_translation_fault_callback,
            /*_request_vaddr*/ block_aligned_vaddr,
            /*_request_size*/ cache_block_size,
            /*_requestor_id*/ requestor_id,
            /*_pc*/ pc,
            /*_ready_tick*/ curTick() + clock_domain->cyclesToTicks(
                request_propagation_delay_in_cycles
            )
        );
    outstanding_requests[block_aligned_vaddr] = bookkeeper;
    paddr_to_vaddr[block_aligned_paddr] = block_aligned_vaddr;
    pending_memory_queue.push_back(bookkeeper);
    bookkeeper->profileQueueEnteringTick();
    stats.num_memory_request_enqueued++;

    // The ruby prefetch proxy will check the pending memory queue and send out
    // requests when they are ready, so we don't need to schedule an event
    // here.
    return true;
}

void
MemoryRequestManager::handleTranslationCompletion(
    MemoryRequestBookkeeper* bookkeeper
)
{
    bookkeeper->ready_tick = std::max(
        bookkeeper->ready_tick,
        curTick() + clock_domain->cyclesToTicks(
            request_propagation_delay_in_cycles
        )
    );
    paddr_to_vaddr[bookkeeper->request_paddr] = bookkeeper->request_vaddr;
    pending_memory_queue.push_back(bookkeeper);
    scheduleSendAddressTranslationRequestsEvent();
}

void
MemoryRequestManager::handleTranslationFault(
    MemoryRequestBookkeeper* bookkeeper, const Fault& fault
)
{
    bookkeeper->ready_tick = std::max(
        bookkeeper->ready_tick,
        curTick() + clock_domain->cyclesToTicks(
            request_propagation_delay_in_cycles
        )
    );
    completed_request_queue.push(
        /*priority*/ bookkeeper->ready_tick,
        /*key*/ bookkeeper->request_paddr,
        /*value*/ bookkeeper
    );
    scheduleProcessCompletedRequestQueueEvent();
}

void
MemoryRequestManager::errorIfTranslationComplete(
    MemoryRequestBookkeeper* bookkeeper
)
{
    panic(
        "Address translation completed for a request bookkeeper that is "
        "configured to not have address translation."
    );
}

void
MemoryRequestManager::errorIfTranslationFault(
    MemoryRequestBookkeeper* bookkeeper, const Fault& fault
)
{
    panic(
        "Address translation completed for a request bookkeeper that is "
        "configured to not have address translation."
    );
}

bool
MemoryRequestManager::hasPendingMemoryRequests() const
{
    return !pending_memory_queue.empty();
}

Tick
MemoryRequestManager::getNextReadyRequestTick() const
{
    if (!hasPendingMemoryRequests()) {
        return MaxTick;
    }
    return pending_memory_queue.front()->ready_tick;
}

PacketPtr
MemoryRequestManager::getNextRequestPacket()
{
    processLocalCacheHitsFromPendingMemoryQueue();

    if (!hasPendingMemoryRequests()) {
        return nullptr;
    }
    // We don't check if the request is ready to be issued here, as the
    // prefetcher proxy will check that before calling getNextRequestPacket.
    // We just return the packet of the next request to be issued.
    MemoryRequestBookkeeper* bookkeeper = pending_memory_queue.front();
    if (!bookkeeper->isReady()) {
        // If the next request is not ready yet, we return nullptr and wait for
        // the prefetcher proxy to call getNextRequestPacket again when the
        // request is ready.
        owner->recheckPendingPrefetchRequests();
        return nullptr;
    }
    pending_memory_queue.pop_front();
    stats.num_memory_request_issued++;
    processLocalCacheHitsFromPendingMemoryQueue();
    return bookkeeper->getPacket();
}

void
MemoryRequestManager::processLocalCacheHitsFromPendingMemoryQueue()
{
    bool has_new_completed_request = false;
    while (true) {
        if (pending_memory_queue.empty()) {
            break;
        }
        MemoryRequestBookkeeper* bookkeeper = pending_memory_queue.front();
        if (bookkeeper->isCompleted()) {
            // Just to make sure we don't process the same completed request
            // multiple times.
            pending_memory_queue.pop_front();
            continue;
        }
        const Addr block_aligned_paddr = bookkeeper->request_paddr;
        ruby::CHI::Cache_CacheEntry* entry =
            cache_controller->getCacheEntry(block_aligned_paddr);
        if (entry == nullptr) {
            break;
        }
        if (completed_request_queue.contains(block_aligned_paddr)) {
            // This means that the request has already been marked as completed
            // and is waiting in the completed request queue to be processed,
            // so we don't need to process it again.
            pending_memory_queue.pop_front();
            continue;
        }
        DMP_MEMORY_MANAGER_DEBUG(
            "Memory request for paddr 0x%llx hits in local cache. "
            "Marking the request as completed and notifying the prefetch "
            "queue.\n",
            block_aligned_paddr
        );
        stats.num_local_cache_hits++;
        pending_memory_queue.pop_front();
        const ruby::DataBlock& response_data = entry->getDataBlk();
        bookkeeper->setDataFromDataBlock(response_data, cache_block_size);
        bookkeeper->local_cache_hit = true;
        // We serialize the access here. We only allow one local cache access
        // at a time, and the next local cache access can only start after the
        // current one finishes.
        const Tick current_tick = curTick();
        const Tick access_delay = clock_domain->cyclesToTicks(
            local_cache_data_access_delay_in_cycles
        );
        Tick ready_tick_after_local_cache_access = 0;
        if (current_tick < previous_local_cache_access_completion_tick) {
            // if the current tick is still before the completion of the
            // previous local cache access, we have to wait until the previous
            // local cache access completes before we can access the local
            // cache for this request.
            ready_tick_after_local_cache_access =
                previous_local_cache_access_completion_tick + access_delay;
        } else {
            ready_tick_after_local_cache_access = current_tick + access_delay;
        }
        bookkeeper->ready_tick = std::max(
            bookkeeper->ready_tick + access_delay,
            ready_tick_after_local_cache_access
        );
        previous_local_cache_access_completion_tick = bookkeeper->ready_tick;
        completed_request_queue.push(
            /*priority*/ bookkeeper->ready_tick,
            /*key*/ bookkeeper->request_paddr,
            /*value*/ bookkeeper
        );
        has_new_completed_request = true;
    }

    if (has_new_completed_request) {
        scheduleProcessCompletedRequestQueueEvent();
    }
}

void
MemoryRequestManager::processMemoryResponse(PacketPtr pkt)
{
    const Addr paddr = pkt->req->getPaddr();
    const Addr block_aligned_paddr = paddr & ~(cache_block_size - 1);
    assert(block_aligned_paddr % cache_block_size == 0);
    const Addr vaddr = pkt->req->getVaddr();
    const Addr block_aligned_vaddr = vaddr & ~(cache_block_size - 1);

    DMP_MEMORY_MANAGER_DEBUG(
        "Processing memory response for paddr 0x%llx, vaddr 0x%llx\n",
        block_aligned_paddr, block_aligned_vaddr
    );
    auto bookkeeper_it = outstanding_requests.find(block_aligned_vaddr);
    if (bookkeeper_it == outstanding_requests.end()) {
        // This should never happen, as we should only receive memory responses
        // for requests that we have sent out, and all sent out requests should
        // be in translated_outstanding_requests.
        DMP_MEMORY_MANAGER_DEBUG(
            "Received memory response for paddr 0x%llx, but no outstanding "
            "request found for this address.\n",
            block_aligned_vaddr
        );
        paddr_to_vaddr.erase(block_aligned_paddr);
        return;
    }
    MemoryRequestBookkeeper* bookkeeper = bookkeeper_it->second;
    if (!bookkeeper->isReady()) {
        // Should not happen as the prefetcher proxy only receives new
        // prefetch requests from the MemoryRequestManager when they are ready.
        return;
    }
    if (bookkeeper->isCompleted()) {
        // This can happen when the request hits in the local cache and is
        // marked as completed, then that block is evicted from the local cache
        // before being pulled in again.
        return;
    }

    if (completed_request_queue.contains(block_aligned_paddr)) {
        // Similar to the case above. Typically, a bookkeeper is marked as
        // completed and put in the completed request queue at the same time.
        // If we reach here, probably there's a bug.
        return;
    }

    // copy the data as it will be deleted after this function returns
    bookkeeper->ready_tick = std::max(bookkeeper->ready_tick, curTick());
    bookkeeper->setDataFromPacket(pkt);
    completed_request_queue.push(
        /*priority*/ bookkeeper->ready_tick,
        /*key*/ block_aligned_paddr,
        /*value*/ bookkeeper
    );

    DMP_MEMORY_MANAGER_DEBUG(
        "Move memory request for paddr 0x%llx, vaddr 0x%llx to completed "
        "request queue, ready tick %lld\n",
        block_aligned_paddr, block_aligned_vaddr, bookkeeper->ready_tick
    );

    scheduleProcessCompletedRequestQueueEvent();
}

void
MemoryRequestManager::processPendingTranslationQueue()
{
    // We send 1 address translation request per cycle.
    if (!pending_translation_queue.empty()) {
        auto bookkeeper = pending_translation_queue.front();
        if (!bookkeeper->isReady()) {
            // The requests in the pending translation queue are ordered by
            // their ready ticks, so if the front request is not ready yet,
            // then the rest of the requests in the queue are also not ready
            // yet, and we can stop processing the pending translation queue
            // for now.
        } else {
            pending_translation_queue.pop();
            mmu->translateTiming(
                /*req*/ bookkeeper->getRequest(),
                /*tc*/ thread_context,
                /*translation*/ new AddressTranslationHandler(
                    /*_bookkeeper*/ bookkeeper,
                    /*_requestor_id*/ requestor_id
                ),
                /*mode*/ BaseMMU::Read
            );
            bookkeeper->profileTranslationStartingTick();
        }
    }
    // Schedule sending the next address translation request if there are still
    // pending requests in the pending translation queue.
    if (!pending_translation_queue.empty()) {
        scheduleSendAddressTranslationRequestsEvent();
    }
}

void
MemoryRequestManager::processCompletedRequestQueue()
{
    while (!completed_request_queue.empty()) {
        auto [ready_tick, key, bookkeeper] = completed_request_queue.top();
        if (ready_tick > curTick()) {
            // The requests in the completed request queue are ordered by their
            // ready ticks, so if the front request is not ready yet, then the
            // rest of the requests in the queue are also not ready yet, and we
            // can stop processing the completed request queue for now.
            break;
        }
        assert(bookkeeper != nullptr);
        completed_request_queue.pop();
        stats.num_memory_request_completed++;
        bookkeeper->profileQueueLeavingTick();
        stats.memory_request_queueing_duration_histogram.sample(
            bookkeeper->getQueueingDuration()
        );
        owner->processCompletedPrefetchRequest(
            /*prefetch_vaddr_block_aligned*/ bookkeeper->request_vaddr,
            /*response_data*/ bookkeeper->response_data,
            /*is_prefetch_hit_in_local_cache*/ bookkeeper->local_cache_hit,
            /*translation_fault*/ bookkeeper->translation_fault
        );
        DMP_MEMORY_MANAGER_DEBUG(
            "Removing completed request for vaddr 0x%llx from completed "
            "request queue, ptr_address 0x%llx\n",
            bookkeeper->request_vaddr, (uint64_t)bookkeeper
        );
        paddr_to_vaddr.erase(bookkeeper->request_paddr);
        outstanding_requests.erase(bookkeeper->request_vaddr);
        pending_memory_queue.erase(std::remove_if(
            pending_memory_queue.begin(), pending_memory_queue.end(),
            [bookkeeper](MemoryRequestBookkeeper* bk) {
                return bk == bookkeeper;
            }
        ), pending_memory_queue.end());
        delete bookkeeper;
    }
    if (!completed_request_queue.empty()) {
        scheduleProcessCompletedRequestQueueEvent();
    }
}

void
MemoryRequestManager::scheduleSendAddressTranslationRequestsEvent()
{
    const bool event_already_scheduled =
        process_pending_translation_queue_event.scheduled();
    const bool has_pending_translation = !pending_translation_queue.empty();
    if (!event_already_scheduled && has_pending_translation) {
        const Tick scheduled_tick =
            std::max(
                pending_translation_queue.front()->ready_tick,
                curTick() + clock_domain->cyclesToTicks(Cycles(1))
            );
        owner->schedule(
            process_pending_translation_queue_event, scheduled_tick
        );
    }
}

void
MemoryRequestManager::scheduleProcessCompletedRequestQueueEvent()
{
    const bool event_already_scheduled =
        process_completed_request_event.scheduled();
    const bool has_completed_request = !completed_request_queue.empty();
    if (has_completed_request) {
        auto [ready_tick, paddr, bookkeeper] = completed_request_queue.top();
        const Tick next_ready_tick = std::max(
            curTick() + clock_domain->cyclesToTicks(Cycles(1)),
            ready_tick
        );
        if (!event_already_scheduled) {
            owner->schedule(process_completed_request_event, next_ready_tick);
        } else {
            owner->reschedule(
                process_completed_request_event, next_ready_tick
            );
        }
    }
}

MemoryRequestManager::MemoryRequestManagerStats::MemoryRequestManagerStats(
    PrefetchQueue* _parent, MemoryRequestManager* _owner,
    ClockDomain* _clock_domain
) : statistics::Group(_parent, "MemoryRequestManagerStats"),
    parent(_parent),
    owner(_owner),
    clock_domain(_clock_domain),
    ADD_STAT(
        num_memory_request_enqueued, statistics::units::Count::get(),
        "Number of memory requests enqueued to the memory system"
    ),
    ADD_STAT(
        num_memory_request_issued, statistics::units::Count::get(),
        "Number of memory requests issued to the memory system"
    ),
    ADD_STAT(
        num_local_cache_hits, statistics::units::Count::get(),
        "Number of memory requests that hit in the local cache"
    ),
    ADD_STAT(
        num_memory_request_completed, statistics::units::Count::get(),
        "Number of memory requests that are completed (either hit in the "
        "local cache or received a response from the memory system)"
    ),
    ADD_STAT(
        num_memory_request_failed_due_to_translation_fault,
        statistics::units::Count::get(),
        "Number of memory requests that failed due to translation fault"
    ),
    ADD_STAT(
        memory_request_queueing_duration_histogram,
         statistics::units::Tick::get(),
        "Histogram of memory request queuing duration."
    ),
    ADD_STAT(
        num_memory_requests_stuck, statistics::units::Count::get(),
        "Number of memory requests that got stuck for more than 10000 cycles "
        "and never got completed"
    ),
    ADD_STAT(
        memory_request_stuck_duration_histogram,
        statistics::units::Tick::get(),
        "Histogram of memory request latency for requests that are fulfilled "
        "by the memory system"
    )

{
}

void
MemoryRequestManager::MemoryRequestManagerStats::regStats()
{
    statistics::Group::regStats();

    memory_request_queueing_duration_histogram
        .init(16)
        .flags(statistics::pdf);
    memory_request_stuck_duration_histogram
        .init(16)
        .flags(statistics::pdf);
}

void
MemoryRequestManager::MemoryRequestManagerStats::preDumpStats()
{
    statistics::Group::preDumpStats();

    DMP_MEMORY_MANAGER_STUCK_DEBUG(
        "Predump stats for MemoryRequestManager %s\n", parent->name()
    );
    DMP_MEMORY_MANAGER_STUCK_DEBUG("Current tick: %lu\n", curTick());

    const Tick cur_tick = curTick();
    const Tick threshold = clock_domain->cyclesToTicks(Cycles(10000));
    for (
        auto &[block_aligned_vaddr, bookkeeper] : owner->outstanding_requests
    ) {
        const Tick request_duration =
            cur_tick - bookkeeper->getQueueEnteringTick();
        if (request_duration > threshold) {
            num_memory_requests_stuck++;
            memory_request_stuck_duration_histogram.sample(
                request_duration
            );
            DMP_MEMORY_MANAGER_STUCK_DEBUG(
                "Memory request for vaddr 0x%llx has been outstanding since "
                "tick %lld, which exceeds the threshold of %lld ticks. This "
                "request is considered stuck.\n",
                block_aligned_vaddr, bookkeeper->getQueueEnteringTick(),
                threshold
            );
        }
    }
}

} // namespace dmp

} // namespace prefetch

} // namespace gem5
