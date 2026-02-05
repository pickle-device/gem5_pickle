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
#include <list>
#include <queue>

#include "arch/generic/mmu.hh"
#include "base/logging.hh"
#include "base/types.hh"
#include "debug/DifferentialMatchingPrefetcherMemoryRequestManagerDebug.hh"
#include "mem/cache/prefetch/differential_matching_prefetcher/prefetch_request.hh"
#include "mem/request.hh"

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
    const Addr vaddr;
    const Addr paddr;
    // The earliest time when the memory request can be issued
    const Tick earliest_issue_tick;
    // Don't use the constructor directly.
    // Use the factory method in MemoryRequestManager instead.
    MemoryRequestBookkeeper(
      const Addr _vaddr, const Addr _paddr, const Tick _earliest_issue_tick
    );
    ~MemoryRequestBookkeeper() = default;
    // Factory method to create a MemoryRequestBookkeeper
    static MemoryRequestBookkeeper* createPrefetchRequestUsingVirtualAddr(
        const Addr _vaddr, const Tick _earliest_issue_tick
    );
    static MemoryRequestBookkeeper* createPrefetchRequestUsingPhysicalAddr(
        const Addr _paddr, const Tick _earliest_issue_tick
    );
};  // class MemoryRequestBookkeeper

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
    RequestorID requestor_id;
    BaseMMU* mmu;
    Cycles request_propagation_delay;
    const bool skip_address_translation;
    // Mapping from block-aligned address to outstanding memory request
    // bookkeeper. When skip_address_translation is false, the key is the
    // block-aligned virtual address; otherwise, it is the block-aligned
    // physical address.
    // The bookkeeper tracks the state of the memory request, and the set
    // of bookkeepers in outstanding_requests is the union of requests that are
    // pending translation and requests that are pending memory issue.
    std::unordered_map<Addr, MemoryRequestBookkeeper*> outstanding_requests;
    // Requests that are ready for address translation, but have not yet
    // started address translation.
    std::queue<MemoryRequestBookkeeper*> pending_translation_queue;
    // Requests that are ready to be issued to the memory system, but have not
    // yet been issued.
    std::queue<MemoryRequestBookkeeper*> pending_memory_queue;
  public:
    MemoryRequestManager(
        PrefetchQueue* _owner, const RequestorID _requestor_id, BaseMMU* _mmu,
        const Cycles _request_propagation_delay
    );

    // Return:
    // - true if the prefetch request is successfully enqueued,
    // - false if the request manager already has an outstanding request
    //   for the same cache block address.
    bool enqueuePrefetchRequestUsingVirtualAddr(Addr block_aligned_vaddr);
    bool enqueuePrefetchRequestUsingPhysicalAddr(Addr block_aligned_paddr);
};  // class MemoryRequestManager

} // namespace dmp

} // namespace prefetch

} // namespace gem5

#endif // __DMP_MEMORY_REQUEST_MANAGER_HH__
