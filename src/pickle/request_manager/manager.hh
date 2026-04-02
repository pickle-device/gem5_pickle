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
**/

#ifndef __PICKLE_DEVICE_REQUEST_MANAGER_HH__
#define __PICKLE_DEVICE_REQUEST_MANAGER_HH__

#include <memory>
#include <queue>

#include "arch/generic/mmu.hh"
#include "base/statistics.hh"
#include "debug/PickleDeviceRequestManagerDebug.hh"
#include "params/PickleDeviceRequestManager.hh"
#include "pickle/request_manager/helpers.hh"
#include "sim/eventq.hh"
#include "sim/sim_object.hh"
#include "sim/system.hh"

namespace gem5
{

class PickleDevice;

class PickleDeviceRequestManager : public SimObject
{
    public:
        PickleDeviceRequestManager(
            const PickleDeviceRequestManagerParams &params
        );
        ~PickleDeviceRequestManager();
    public:
        void switchOn();
        void switchOff();
        // Enqueue a load request.
        // Returns true if the request is successfully enqueued, false if the
        // request manager is not activated.
        //  - vaddr: the virtual address of the load request
        //  - context_id: the context ID of the load request. This is used for
        // address translation if the request manager is configured to perform
        // address translation on the core's MMUs (not the PickleMMU)
        //  - only_complete_address_translation: if true, the request manager
        //               only performs address translation for the request, and
        //               does not send the data request to the cache hierarchy
        bool enqueueLoadRequest(
            const Addr vaddr, const ContextID context_id,
            bool only_complete_address_translation
        );
        // Enqueue a store request.
        // Returns true if the request is successfully enqueued, false if the
        // request manager is not activated.
        //  - vaddr: the virtual address of the store request
        //  - context_id: the context ID of the load request. This is used for
        // address translation if the request manager is configured to perform
        // address translation on the core's MMUs (not the PickleMMU)
        //  - data_ptr: the data pointer for the store request
        //  - only_complete_address_translation: if true, the request manager
        //               only performs address translation for the request, and
        //               does not send the data request to the cache hierarchy
        bool enqueueStoreRequest(
            const Addr vaddr, const ContextID context_id,
            std::unique_ptr<uint8_t*> data_ptr,
            bool only_complete_address_translation
        );
        // Add a request to the request manager.
        //   - vaddr: the virtual address of the request
        //  - context_id: the context ID of the load request. This is used for
        // address translation if the request manager is configured to perform
        // address translation on the core's MMUs (not the PickleMMU)
        //   - is_load: true if the request is a load, false if it is a store
        //   - data_ptr: the data pointer for store requests, nullptr for load
        //               requests
        //  - only_complete_address_translation: if true, the request manager
        //               only performs address translation for the request, and
        //               does not send the data request to the cache hierarchy
        bool enqueueRequest(
            const Addr vaddr, const ContextID context_id, bool is_load,
            std::unique_ptr<uint8_t*> data_ptr,
            bool only_complete_address_translation
        );
        void setRequestorID(const RequestorID requestor_id);
        void setMMU(BaseMMU* mmu);
        void setFunctionalMMU(BaseMMU* functional_mmu);
        void setOwner(PickleDevice* owner);
        void handleRequestCompletion(PacketPtr pkt);
    private:
        bool is_activated;
        PickleDevice* owner;
        BaseMMU* mmu;
        BaseMMU* functional_mmu;
        RequestorID requestor_id;
        uint64_t ticks_per_cycle;
        System *system;
        bool use_core_mmus_for_address_translation;
        bool use_functional_mmu;
        Tick pickle_to_core_mmus_latency_in_ticks;
        void handleTranslationCompletion(
            std::shared_ptr<RequestBookkeeper> request_bookkeeper
        );
        void handleTranslationFault(
            std::shared_ptr<RequestBookkeeper> request_bookkeeper,
            const Fault& fault
        );
        static constexpr uint64_t BLOCK_SHIFT = 6;
        static constexpr uint64_t PAGE_SHIFT = 12;
        static constexpr uint64_t BLOCK_SIZE = (1 << BLOCK_SHIFT);
        static constexpr uint64_t PAGE_SIZE = (1 << PAGE_SHIFT);
        // block_aligned_vaddr -> list of request bookkeepers
        std::unordered_map< \
            Addr, std::vector<std::shared_ptr<RequestBookkeeper>> \
        > outstanding_requests;
        std::queue<std::shared_ptr<RequestBookkeeper>> \
            need_retry_handle_translation_completion;
        EventFunctionWrapper retry_handle_translation_completion_event;
        void retryHandleTranslationCompletion();
        void addRetryHandleTranslationCompletion(
            std::shared_ptr<RequestBookkeeper> request_bookkeeper
        );
        void removeOutstandingRequestViaPacketPtr(PacketPtr pkt);
        void removeOutstandingRequestViaRequestBookkeeper(
            std::shared_ptr<RequestBookkeeper> request_bookkeeper
        );
        void profileRequest(const Addr vaddr);
        void profileRequestCoalescing(const Addr vaddr);
    public:
        // stats
        struct PickleDeviceRequestManagerStats : public statistics::Group
        {
            PickleDeviceRequestManagerStats(statistics::Group *parent);
            statistics::Scalar numRequestsReceivedFromOwner;
            statistics::Scalar numRequestInitiatedAfterTranslation;
            statistics::Scalar numRequestsCompleted;
            statistics::Scalar numTranslationFaults;
            statistics::Histogram requestQueueLength;
            statistics::Scalar requestsCount;
            statistics::Scalar requestsCountAfterCoalescing;
        } request_manager_stats;
}; // class PickleDeviceRequestManager

}; // namespace gem5

#endif // __PICKLE_DEVICE_REQUEST_MANAGER_HH__
