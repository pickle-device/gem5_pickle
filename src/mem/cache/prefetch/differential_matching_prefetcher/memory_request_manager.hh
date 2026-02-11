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
#include <list>
#include <queue>
#include <unordered_map>
#include <utility>

#include "arch/generic/mmu.hh"
#include "base/logging.hh"
#include "base/types.hh"
#include "debug/DifferentialMatchingPrefetcherMemoryRequestManagerDebug.hh"
#include "mem/cache/prefetch/differential_matching_prefetcher/prefetch_request.hh"
#include "mem/packet.hh"
#include "mem/request.hh"
#include "mem/ruby/protocol/CHI/Cache_Controller.hh"
#include "sim/clock_domain.hh"
#include "sim/eventq.hh"

#define DMP_MEMORY_MANAGER_DEBUG(...) \
    DPRINTF(DifferentialMatchingPrefetcherMemoryRequestManagerDebug,\
            "(Memory Manager) " __VA_ARGS__)

namespace gem5
{

namespace prefetch
{

namespace dmp
{

class PrefetchQueue;

class MemoryRequestBookkeeper
{
  public:
    const Addr request_vaddr;
    const Addr request_paddr;
    const uint64_t request_size;
    const RequestorID requestor_id;
    const Addr pc;
    bool local_cache_hit;
    // The earliest time when the memory request is ready, i.e., can be issued
    Tick ready_tick;
    // Response data
    std::vector<uint8_t> response_data;
    // Don't use the constructor directly.
    // Use the factory method in MemoryRequestManager instead.
    MemoryRequestBookkeeper(
      const Addr _request_vaddr, const Addr _request_paddr,
      const uint64_t _request_size, const RequestorID _requestor_id,
      const Addr _pc, const Tick _ready_tick, const bool has_physical_address
    );
    ~MemoryRequestBookkeeper();
    // Factory method to create a MemoryRequestBookkeeper
    static MemoryRequestBookkeeper* createPrefetchRequestUsingVirtualAddr(
      const Addr _request_vaddr, const uint64_t _request_size,
      const RequestorID _requestor_id, const Addr _pc, const Tick _ready_tick
    );
    static MemoryRequestBookkeeper* createPrefetchRequestUsingPhysicalAddr(
      const Addr _request_paddr, const uint64_t _request_size,
      const RequestorID _requestor_id, const Addr _pc, const Tick _ready_tick
    );
    RequestPtr getRequest();
    PacketPtr getPacket();
    bool hasPhysicalAddress() const;
    void setDataFromPacket(PacketPtr pkt);
    void setDataFromDataBlock(
      const ruby::DataBlock& data_block, const uint64_t cache_block_size
    );
    // Return true if the memory request has received response and is already
    // in the completed request queue, waiting for the prefetch queue to be
    // notified. Return false otherwise.
    bool isCompleted() const;
    bool isReady() const;

  private:
    RequestPtr request;
    PacketPtr packet;
    bool has_physical_address;
};  // class MemoryRequestBookkeeper

struct ReadyTickMemoryRequestComparator
{
    bool operator()(
        const std::pair<Tick, MemoryRequestBookkeeper*>& a,
        const std::pair<Tick, MemoryRequestBookkeeper*>& b
    ) const {
        // The request with the smaller ready tick should have higher priority.
        return a.first > b.first;
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
    std::unordered_map<Addr, MemoryRequestBookkeeper*> outstanding_requests;

    // Mapping physical address to virtual address of the outstanding requests.
    // Used to map the memory response back to the outstanding request.
    // Should not be used as an address translation buffer.
    std::unordered_map<Addr, Addr> paddr_to_vaddr;

    // Requests that are ready for address translation, but have not yet
    // started address translation.
    std::queue<MemoryRequestBookkeeper*> pending_translation_queue;
    // Requests that are ready to be issued to the memory system, but have not
    // yet been issued.
    std::deque<MemoryRequestBookkeeper*> pending_memory_queue;
    // Requests that have been completed (either successfully or
    // unsuccessfully), but the prefetch queue has not yet been notified.
    std::priority_queue<
      std::pair<Tick, MemoryRequestBookkeeper*>,
      std::vector<std::pair<Tick, MemoryRequestBookkeeper*>>,
      ReadyTickMemoryRequestComparator
    > completed_request_queue;

    // Event handlers
    EventFunctionWrapper process_pending_translation_queue_event;
    EventFunctionWrapper process_completed_request_event;

  public:
    MemoryRequestManager(
      PrefetchQueue* _owner, ClockDomain* _clock_domain,
      uint64_t _cache_block_size, const RequestorID _requestor_id,
      BaseMMU* _mmu, const Cycles _local_cache_data_access_delay_in_cycles,
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
};  // class MemoryRequestManager

} // namespace dmp

} // namespace prefetch

} // namespace gem5

#endif // __DMP_MEMORY_REQUEST_MANAGER_HH__
