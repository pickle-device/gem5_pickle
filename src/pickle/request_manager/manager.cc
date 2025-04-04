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

#include "pickle/request_manager/manager.hh"

#include "pickle/device/pickle_device.hh"

namespace gem5
{

PickleDeviceRequestManager::PickleDeviceRequestManager(
    const PickleDeviceRequestManagerParams &params
) : SimObject(params),
    is_activated(false),
    owner(nullptr),
    mmu(nullptr),
    retry_handle_translation_completion_event(
        [this]{retryHandleTranslationCompletion();}, name() + ".retry_event"
    ),
    request_manager_stats(this)
{
}

PickleDeviceRequestManager::~PickleDeviceRequestManager()
{
}

void
PickleDeviceRequestManager::switchOn()
{
    is_activated = true;
}

void
PickleDeviceRequestManager::switchOff()
{
    is_activated = false;
}

bool
PickleDeviceRequestManager::enqueueLoadRequest(const Addr vaddr)
{
    // Enqueue a load request to the request manager.
    return enqueueRequest(vaddr, true, nullptr);
}

bool
PickleDeviceRequestManager::enqueueStoreRequest(
    const Addr vaddr, std::unique_ptr<uint8_t*> data_ptr
)
{
    assert(data_ptr != nullptr);
    return enqueueRequest(vaddr, false, std::move(data_ptr));
}

bool
PickleDeviceRequestManager::enqueueRequest(
    const Addr vaddr, bool is_load, std::unique_ptr<uint8_t*> data_ptr
)
{
    request_manager_stats.numRequestsReceivedFromOwner++;
    request_manager_stats.requestQueueLength.sample(
        outstanding_requests.size()
    );
    DPRINTF(PickleDeviceRequestManagerDebug,
        "enqueueRequest: vaddr = 0x%llx, isLoad = %d\n", vaddr, is_load
    );
    Addr block_aligned_vaddr = (vaddr >> BLOCK_SHIFT) << BLOCK_SHIFT;
    profileRequest(block_aligned_vaddr);
    bool already_requested = false;
    if (outstanding_requests.find(block_aligned_vaddr) != \
            outstanding_requests.end()) {
        already_requested = true;
    }
    outstanding_requests[block_aligned_vaddr] = \
        std::vector<std::shared_ptr<RequestBookkeeper>>();

    // If the request is a load and the block has already been requested,
    // we ignore the request, aka coalescing.
    if (is_load && already_requested) {
        profileRequestCoalescing(block_aligned_vaddr);
        return true;
    }

    Request::Flags flags = 0;
    RequestPtr req = std::make_shared<Request>(
        block_aligned_vaddr, BLOCK_SIZE, flags, requestor_id, 0, 0
    );

    static AddressTranslationDoneCallbackType done_callback = std::bind(
        &PickleDeviceRequestManager::handleTranslationCompletion,
        this, std::placeholders::_1
    );

    static AddressTranslationFaultCallbackType fault_callback = std::bind(
        &PickleDeviceRequestManager::handleTranslationFault,
        this, std::placeholders::_1, std::placeholders::_2
    );

    // Add the request
    outstanding_requests[block_aligned_vaddr].emplace_back(
        new RequestBookkeeper(
            done_callback, fault_callback, req, is_load, std::move(data_ptr)
        )
    );

    mmu->translateTiming(
        req, owner->getThreadContextPtr(), new PickleDeviceAddressTranslation(
            outstanding_requests[block_aligned_vaddr].back(), requestor_id
        ),
        BaseMMU::Read
    );

    DPRINTF(PickleDeviceRequestManagerDebug,
        "Started translation for vaddr 0x%llx\n", block_aligned_vaddr
    );

    return true;
}

void
PickleDeviceRequestManager::handleTranslationCompletion(
    std::shared_ptr<RequestBookkeeper> request_bookkeeper
)
{
    // When the translation is done, we'll send the request to the cache
    // hierarchy.
    DPRINTF(PickleDeviceRequestManagerDebug,
        "Translation done for vaddr 0x%llx, paddr 0x%llx\n",
        request_bookkeeper->getPkt()->req->getVaddr(),
        request_bookkeeper->getPkt()->req->getPaddr()
    );
    bool success = owner->request_port.sendTimingReq(
        request_bookkeeper->getPkt()
    );
    if (success) {
        request_bookkeeper->translationSent();
        request_manager_stats.numRequestInitiatedAfterTranslation++;;
        DPRINTF(PickleDeviceRequestManagerDebug,
            "Done translation for vaddr 0x%llx\n",
            request_bookkeeper->getReq()->getVaddr()
        );
    } else {
        addRetryHandleTranslationCompletion(request_bookkeeper);
        DPRINTF(PickleDeviceRequestManagerDebug,
            "Retrying translation completion for vaddr 0x%llx\n",
            request_bookkeeper->getReq()->getVaddr()
        );
    }
}

void
PickleDeviceRequestManager::addRetryHandleTranslationCompletion(
    std::shared_ptr<RequestBookkeeper> request_bookkeeper
)
{
    // When the translation is done, we'll send the request to the cache
    // hierarchy.
    need_retry_handle_translation_completion.push(request_bookkeeper);
    if (!retry_handle_translation_completion_event.scheduled()) {
        schedule(
            retry_handle_translation_completion_event,
            curTick() + 1000
        );
        DPRINTF(PickleDeviceRequestManagerDebug,
            "Scheduled retrying translation completion for vaddr 0x%llx\n",
            request_bookkeeper->getReq()->getVaddr()
        );
    }
    DPRINTF(PickleDeviceRequestManagerDebug,
        "Add retrying translation completion for vaddr 0x%llx\n",
        request_bookkeeper->getReq()->getVaddr()
    );
}

void
PickleDeviceRequestManager::retryHandleTranslationCompletion()
{
    // When the translation is done, we'll send the request to the cache
    // hierarchy.
    while (!need_retry_handle_translation_completion.empty()) {
        auto request_bookkeeper = \
            need_retry_handle_translation_completion.front();
        bool success = owner->request_port.sendTimingReq(
            request_bookkeeper->getPkt()
        );
        if (!success) {
            schedule(
                retry_handle_translation_completion_event,
                curTick() + 1000
            );
            DPRINTF(PickleDeviceRequestManagerDebug,
                "Retrying translation completion for vaddr 0x%llx\n",
                request_bookkeeper->getReq()->getVaddr()
            );
            break;
        }
        need_retry_handle_translation_completion.pop();
        request_manager_stats.numRequestInitiatedAfterTranslation++;
        request_bookkeeper->requestPending();
        DPRINTF(PickleDeviceRequestManagerDebug,
            "Retrying translation completion for vaddr 0x%llx\n",
            request_bookkeeper->getReq()->getVaddr()
        );
    }
}

void
PickleDeviceRequestManager::handleTranslationFault(
    std::shared_ptr<RequestBookkeeper> request_bookkeeper, const Fault& fault
)
{
    // When the translation fails, we'll notify the requestor.
    // TODO: remove the request from the outstanding requests.
    request_manager_stats.numTranslationFaults++;
    request_manager_stats.requestQueueLength.sample(
        outstanding_requests.size()
    );
    DPRINTF(PickleDeviceRequestManagerDebug,
        "Translation fault for vaddr 0x%llx\n", request_bookkeeper->getVAddr()
    );
    Addr vaddr = request_bookkeeper->getVAddr();
    owner->handleRequestTranslationFault(vaddr);
    if (outstanding_requests.find(vaddr) == outstanding_requests.end()) {
        return;
    }
    // Remove the request from the outstanding requests.
    removeOutstandingRequestViaRequestBookkeeper(request_bookkeeper);
    owner->device_stats.numTranslationFaults++;
}

void
PickleDeviceRequestManager::setOwner(PickleDevice* owner)
{
    this->owner = owner;
    DPRINTF(PickleDeviceRequestManagerDebug,
        "Set owner to %s\n", owner->name()
    );
}

void
PickleDeviceRequestManager::setMMU(BaseMMU* mmu)
{
    this->mmu = mmu;
    DPRINTF(PickleDeviceRequestManagerDebug,
        "Set MMU to %s\n", mmu->name()
    );
}

void
PickleDeviceRequestManager::setRequestorID(const RequestorID requestor_id)
{
    this->requestor_id = requestor_id;
    DPRINTF(PickleDeviceRequestManagerDebug,
        "Set requestor id to %d\n", requestor_id
    );
}

void
PickleDeviceRequestManager::handleRequestCompletion(PacketPtr pkt)
{
    request_manager_stats.numRequestsCompleted++;
    request_manager_stats.requestQueueLength.sample(
        outstanding_requests.size()
    );
    Addr vaddr = pkt->req->getVaddr();
    Addr block_aligned_vaddr = (vaddr >> BLOCK_SHIFT) << BLOCK_SHIFT;
    removeOutstandingRequestViaPacketPtr(pkt);

    // When the request is done, we'll notify the requestor.
    DPRINTF(PickleDeviceRequestManagerDebug,
        "Request done for vaddr 0x%llx\n", block_aligned_vaddr
    );
}

void
PickleDeviceRequestManager::removeOutstandingRequestViaPacketPtr(PacketPtr pkt)
{
    Addr vaddr = pkt->req->getVaddr();
    Addr block_aligned_vaddr = (vaddr >> BLOCK_SHIFT) << BLOCK_SHIFT;
    if (outstanding_requests.find(block_aligned_vaddr) == \
            outstanding_requests.end()) {
        return;
    }
    outstanding_requests[block_aligned_vaddr].erase(
        std::remove_if(
            outstanding_requests[block_aligned_vaddr].begin(),
            outstanding_requests[block_aligned_vaddr].end(),
            [pkt](
                const std::shared_ptr<RequestBookkeeper>& rbk
            ) {
                if (rbk == nullptr || rbk->getPkt() == nullptr) {
                    return false;
                }
                return rbk->getPkt() == pkt;
            }
        ),
        outstanding_requests[block_aligned_vaddr].end()
    );
    if (outstanding_requests[block_aligned_vaddr].empty()) {
        outstanding_requests.erase(block_aligned_vaddr);
    }
}

void
PickleDeviceRequestManager::removeOutstandingRequestViaRequestBookkeeper(
    std::shared_ptr<RequestBookkeeper> request_bookkeeper
)
{
    Addr vaddr = request_bookkeeper->getReq()->getVaddr();
    Addr block_aligned_vaddr = (vaddr >> BLOCK_SHIFT) << BLOCK_SHIFT;
    if (outstanding_requests.find(block_aligned_vaddr) == \
            outstanding_requests.end()) {
        return;
    }
    outstanding_requests[block_aligned_vaddr].erase(
        std::remove_if(
            outstanding_requests[block_aligned_vaddr].begin(),
            outstanding_requests[block_aligned_vaddr].end(),
            [request_bookkeeper](
                const std::shared_ptr<RequestBookkeeper>& rbk
            ) {
                if (rbk == nullptr) {
                    return false;
                }
                return rbk == request_bookkeeper;
            }
        ),
        outstanding_requests[block_aligned_vaddr].end()
    );
    if (outstanding_requests[block_aligned_vaddr].empty()) {
        outstanding_requests.erase(block_aligned_vaddr);
    }
}

PickleDeviceRequestManager::
PickleDeviceRequestManagerStats::PickleDeviceRequestManagerStats(
    statistics::Group *parent
) : statistics::Group(parent),
    ADD_STAT(numRequestsReceivedFromOwner, statistics::units::Count::get(),
             "Number of requests received from the owner"),
    ADD_STAT(numRequestInitiatedAfterTranslation,
             statistics::units::Count::get(),
             "Number of requests initiated"),
    ADD_STAT(numRequestsCompleted, statistics::units::Count::get(),
             "Number of requests completed"),
    ADD_STAT(numTranslationFaults, statistics::units::Count::get(),
             "Number of translation faults"),
    ADD_STAT(requestQueueLength, statistics::units::Count::get(),
             "Histogram of the request queue length over time"),
    ADD_STAT(requestsCountPerArray, statistics::units::Count::get(),
             "Number of requests per array"),
    ADD_STAT(requestsCountAfterCoalescingPerArray,
             statistics::units::Count::get(),
             "Number of requests per array after coalescing")
{
    requestQueueLength
      .init(16)
      .flags(statistics::pdf);
    requestsCountPerArray
      .init(16)
      .flags(statistics::nozero | statistics::total);
    requestsCountAfterCoalescingPerArray
      .init(16)
      .flags(statistics::nozero | statistics::total);
}

void
PickleDeviceRequestManager::profileRequest(const Addr vaddr)
{
    uint64_t array_id = owner->getJobDescriptor()->get_array_id(vaddr);
    if (array_id == -1ULL) {
        return;
    }
    request_manager_stats.requestsCountPerArray[array_id]++;
}

void
PickleDeviceRequestManager::profileRequestCoalescing(const Addr vaddr)
{
    uint64_t array_id = owner->getJobDescriptor()->get_array_id(vaddr);
    if (array_id == -1ULL) {
        return;
    }
    request_manager_stats.requestsCountAfterCoalescingPerArray[array_id]++;
    DPRINTF(PickleDeviceRequestManagerDebug,
        "Request coalescing for vaddr 0x%llx, array_id: %lld\n",
        vaddr, array_id
    );
}

}; // namespace gem5
