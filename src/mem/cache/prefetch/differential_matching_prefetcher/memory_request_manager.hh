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

#ifndef __DMP_MEMORY_REQUEST_MANAGER_HH__
#define __DMP_MEMORY_REQUEST_MANAGER_HH__

#include <cstdint>
#include <deque>
#include <functional>
#include <list>
#include <queue>
#include <unordered_map>
#include <utility>

#include "arch/generic/mmu.hh"
#include "base/logging.hh"
#include "base/statistics.hh"
#include "base/stats/group.hh"
#include "base/types.hh"
#include "cpu/thread_context.hh"
#include "debug/DifferentialMatchingPrefetcherMemoryRequestManagerDebug.hh"
#include "debug/DifferentialMatchingPrefetcherMemoryRequestManagerStuckDebug.hh"
#include "mem/cache/prefetch/differential_matching_prefetcher/prefetch_request.hh"
#include "mem/cache/prefetch/differential_matching_prefetcher/util.hh"
#include "mem/packet.hh"
#include "mem/request.hh"
#include "mem/ruby/protocol/CHI/Cache_Controller.hh"
#include "sim/clock_domain.hh"
#include "sim/eventq.hh"

#define DMP_MEMORY_MANAGER_DEBUG(...) \
    DPRINTF( \
      DifferentialMatchingPrefetcherMemoryRequestManagerDebug, "%s: ", \
      owner->name().c_str() \
    ); \
    DPRINTFR(DifferentialMatchingPrefetcherMemoryRequestManagerDebug,\
            "(Memory Manager) " __VA_ARGS__)

#define DMP_MEMORY_MANAGER_STUCK_DEBUG(...) \
    DPRINTF( \
      DifferentialMatchingPrefetcherMemoryRequestManagerStuckDebug, "%s: ", \
      owner->owner->name().c_str() \
    ); \
    DPRINTFR(DifferentialMatchingPrefetcherMemoryRequestManagerStuckDebug,\
            "(Memory Manager Stats) " __VA_ARGS__)

namespace gem5
{

namespace prefetch
{

namespace dmp
{

class PrefetchQueue;
class MemoryRequestBookkeeper;

// Callback when address translation is done without faults.
using AddressTranslationDoneCallbackType = \
    std::function<void(MemoryRequestBookkeeper*)>;
// Callback when address translation is done with faults.
using AddressTranslationFaultCallbackType = \
    std::function<void(MemoryRequestBookkeeper*, const Fault&)>;

class MemoryRequestBookkeeper
{
  public:
    // Translation result handlers
    // The translation done callback is called when the address translation is
    // finished without faults.
    // The translation fault callback is called when the address translation is
    // finished with faults.
    AddressTranslationDoneCallbackType translation_done_callback;
    AddressTranslationFaultCallbackType translation_fault_callback;

    Addr request_vaddr;
    Addr request_paddr;
    uint64_t request_size;
    RequestorID requestor_id;
    Addr pc;
    bool local_cache_hit;
    // The earliest time when the memory request is ready, i.e., can be issued
    Tick ready_tick;
    // Response data
    std::vector<uint8_t> response_data;
    bool translation_fault;

    // Don't use the constructor directly.
    // Use the factory method in MemoryRequestManager instead.
    MemoryRequestBookkeeper(
      AddressTranslationDoneCallbackType _translation_done_callback,
      AddressTranslationFaultCallbackType _translation_fault_callback,
      const Addr _request_vaddr, const Addr _request_paddr,
      const uint64_t _request_size, const RequestorID _requestor_id,
      const Addr _pc, const Tick _ready_tick, const bool has_physical_address
    );
    ~MemoryRequestBookkeeper();
    // Factory method to create a MemoryRequestBookkeeper
    static MemoryRequestBookkeeper* createPrefetchRequestUsingVirtualAddr(
      AddressTranslationDoneCallbackType _translation_done_callback,
      AddressTranslationFaultCallbackType _translation_fault_callback,
      const Addr _request_vaddr, const uint64_t _request_size,
      const RequestorID _requestor_id, const Addr _pc, const Tick _ready_tick
    );
    static MemoryRequestBookkeeper* createPrefetchRequestUsingPhysicalAddr(
      AddressTranslationDoneCallbackType _translation_done_callback,
      AddressTranslationFaultCallbackType _translation_fault_callback,
      const Addr _request_paddr, const uint64_t _request_size,
      const RequestorID _requestor_id, const Addr _pc, const Tick _ready_tick
    );
    void profileQueueEnteringTick();
    void profileTranslationStartingTick();
    void profileTranslationFinishingTick();
    void profileQueueLeavingTick();
    Tick getQueueEnteringTick() const;
    Tick getQueueingDuration() const;
    RequestPtr getRequest();
    PacketPtr getPacket();
    bool hasPhysicalAddress() const;
    void setDataFromPacket(PacketPtr pkt);
    void setDataFromDataBlock(
      const ruby::DataBlock& data_block, const uint64_t cache_block_size
    );
    void setTranslationResult(const Fault &fault, const RequestPtr &req);
    // Return true if the memory request has received response and is already
    // in the completed request queue, waiting for the prefetch queue to be
    // notified. Return false otherwise.
    bool isCompleted() const;
    bool isReady() const;

  private:
    Tick queue_entering_tick;
    Tick translation_starting_tick;
    Tick translation_finishing_tick;
    Tick queue_leaving_tick;
    RequestPtr request;
    PacketPtr packet;
    bool has_physical_address;
};  // class MemoryRequestBookkeeper

class AddressTranslationHandler : public BaseMMU::Translation
{
  private:
    MemoryRequestBookkeeper* bookkeeper;
    RequestPtr req;
  public:
    AddressTranslationHandler(
        MemoryRequestBookkeeper* _bookkeeper,
        const RequestorID& _requestor_id
    ) : bookkeeper(_bookkeeper)
    {
        Request::Flags flags;
        Addr vaddr = bookkeeper->request_vaddr;
        req = std::make_shared<Request>(
            /* vaddr */ vaddr,
            /* size */ bookkeeper->request_size,
            /* flags */ flags,
            /* id */ _requestor_id,
            /* pc */ bookkeeper->pc,
            /* context id */ 0
        );
    }

    ~AddressTranslationHandler()
    {
    }

    RequestPtr getRequest() { return req; }

    void markDelayed() override {}

    void finish(
        const Fault &fault, const RequestPtr &req, ThreadContext *tc,
        BaseMMU::Mode mode
    ) override {
        bookkeeper->setTranslationResult(fault, req);
        delete this;
    }
};

struct ReadyTickMemoryRequestComparator
{
    bool operator()(
        const std::tuple<Tick, Addr, MemoryRequestBookkeeper*>& a,
        const std::tuple<Tick, Addr, MemoryRequestBookkeeper*>& b
    ) const {
        // The request with the smaller ready tick should have higher priority.
        return std::get<0>(a) > std::get<0>(b);
    }
};

// This class manages memory requests for the DMP prefetcher.
// - For address translation, it interacts with the core's MMU to translate
//   virtual addresses to physical addresses.
// - For sending out request after address translation, it interacts with the
//   l2 cache controller,
//     - if the cache line is not already present in the cache, a new CHI
//       prefetch request is sent out
//     - if the cache line is already present in the cache, the data is sent
//       back to the prefetch queue
class MemoryRequestManager
{
  private:
    PrefetchQueue* owner;
    ClockDomain* clock_domain;
    uint64_t cache_block_size;
    RequestorID requestor_id;
    ThreadContext* thread_context; // used for address translation
    BaseMMU* mmu;
    // we need cache controller to acquire data in local cache
    ruby::CHI::Cache_Controller* cache_controller;
    // the delay of getting data out of the local cache
    Cycles local_cache_data_access_delay_in_cycles;
    // the delay of sending address translation from the prefetcher (at L1) to
    // the prefetch queue (at L1 for stride prefetcher, at L2 for DMP)
    Cycles request_propagation_delay_in_cycles;
    const bool skip_address_translation;
    Tick previous_local_cache_access_completion_tick;

    // Mapping from block-aligned address to outstanding memory request
    // bookkeeper. When skip_address_translation is false, the key is the
    // block-aligned virtual address; otherwise, it is the block-aligned
    // physical address.
    // The bookkeeper tracks the state of the memory request, and the set
    // of bookkeepers in outstanding_requests is the union of requests that are
    // pending translation, pending memory, and completed request queues.
    // vaddr -> bookkeeper
    std::unordered_map<Addr, MemoryRequestBookkeeper*> outstanding_requests;

    // Mapping physical address to virtual address of the outstanding requests.
    // Used to map the memory response back to the outstanding request.
    // Should not be used as an address translation buffer.
    // paddr -> vaddr
    std::unordered_map<Addr, Addr> paddr_to_vaddr;

    // Requests that are ready for address translation, but have not yet
    // started address translation.
    std::queue<MemoryRequestBookkeeper*> pending_translation_queue;
    // Requests that are ready to be issued to the memory system, but have not
    // yet been issued.
    std::deque<MemoryRequestBookkeeper*> pending_memory_queue;
    // Requests that have been completed (either successfully or
    // unsuccessfully), but the prefetch queue has not yet been notified.
    // paddr -> bookkeeper
    PriorityQueuedDict<
      /*Priority*/ Tick,
      /*Key*/ Addr,
      /*Value*/ MemoryRequestBookkeeper*,
      /*Comparator*/ ReadyTickMemoryRequestComparator
    > completed_request_queue;

    // Event handlers
    EventFunctionWrapper process_pending_translation_queue_event;
    EventFunctionWrapper process_completed_request_event;

    // Callbacks
    AddressTranslationDoneCallbackType default_translation_done_callback;
    AddressTranslationDoneCallbackType panic_if_translation_done_callback;
    AddressTranslationFaultCallbackType default_translation_fault_callback;
    AddressTranslationFaultCallbackType panic_if_translation_fault_callback;
  public:
    MemoryRequestManager(
      PrefetchQueue* _owner, ClockDomain* _clock_domain,
      uint64_t _cache_block_size, const RequestorID _requestor_id,
      ThreadContext* _thread_context, BaseMMU* _mmu,
      const Cycles _local_cache_data_access_delay_in_cycles,
      const Cycles _request_propagation_delay
    );
    void setCacheController(ruby::CHI::Cache_Controller* _cache_controller);

    // Return:
    // - true if the prefetch request is successfully enqueued,
    // - false if the request manager already has an outstanding request
    //   for the same cache block address.
    bool enqueuePrefetchRequestUsingVirtualAddr(
      Addr block_aligned_vaddr, Addr pc
    );
    bool enqueuePrefetchRequestUsingPhysicalAddr(
      Addr block_aligned_paddr, Addr pc
    );

    // Handling address translation
    void handleTranslationCompletion(MemoryRequestBookkeeper* bookkeeper);
    void handleTranslationFault(
      MemoryRequestBookkeeper* bookkeeper, const Fault& fault
    );
    void errorIfTranslationComplete(MemoryRequestBookkeeper* bookkeeper);
    void errorIfTranslationFault(
      MemoryRequestBookkeeper* bookkeeper, const Fault& fault
    );

    bool hasPendingMemoryRequests() const;
    Tick getNextReadyRequestTick() const;
    PacketPtr getNextRequestPacket();

    void processLocalCacheHitsFromPendingMemoryQueue();
    void processMemoryResponse(PacketPtr pkt);

  private:
    // Event handlers
    void processPendingTranslationQueue();
    void processCompletedRequestQueue();
    void scheduleSendAddressTranslationRequestsEvent();
    void scheduleProcessCompletedRequestQueueEvent();

  public:
    struct MemoryRequestManagerStats : public statistics::Group
    {
        PrefetchQueue* parent;
        MemoryRequestManager* owner;
        ClockDomain *clock_domain;
        MemoryRequestManagerStats(
          PrefetchQueue* _parent, MemoryRequestManager* _owner,
          ClockDomain* _clock_domain
        );
        void regStats();
        void preDumpStats();

        statistics::Scalar num_memory_request_enqueued;
        statistics::Scalar num_memory_request_issued;
        statistics::Scalar num_local_cache_hits;
        statistics::Scalar num_memory_request_completed;
        statistics::Scalar num_memory_request_failed_due_to_translation_fault;
        statistics::Histogram memory_request_queueing_duration_histogram;

        statistics::Scalar num_memory_requests_stuck;
        statistics::Histogram memory_request_stuck_duration_histogram;
    } stats;
};  // class MemoryRequestManager

} // namespace dmp

} // namespace prefetch

} // namespace gem5

#endif // __DMP_MEMORY_REQUEST_MANAGER_HH__
