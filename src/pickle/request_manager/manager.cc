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
    retry_handle_translation_completion_event(
        [this]{retryHandleTranslationCompletion();}, name() + ".retry_event"
    )
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
PickleDeviceRequestManager::enqueueRequest(
    const Addr vaddr, bool is_load, std::unique_ptr<uint8_t[]> data_ptr
)
{
    Addr block_aligned_vaddr = (vaddr >> BLOCK_SHIFT) << BLOCK_SHIFT;
    bool already_requested = false;
    if (outstanding_requests.find(vaddr) == outstanding_requests.end()) {
        already_requested = true;
    }
    outstanding_requests[vaddr] = \
        std::vector<std::shared_ptr<RequestBookkeeper>>();

    // If the request is a load and the block has already been requested,
    // we ignore the request.
    if (is_load && already_requested) {
        return true;
    }

    Request::Flags flags = 0;
    MemCmd cmd = is_load ? MemCmd::ReadReq : MemCmd::WriteReq;
    RequestPtr req = std::make_shared<Request>(
        block_aligned_vaddr, BLOCK_SIZE, flags, requestor_id
    );
    PacketPtr pkt = new Packet(req, cmd, BLOCK_SIZE);
    if (is_load) {
        // For load requests, we need to set the data to be received.
        assert(data_ptr == nullptr);
        pkt->dataDynamic<uint8_t>(new uint8_t[BLOCK_SIZE]);
        // The data will be set to the request when the response is received.
        // The pointer will be deleted when the response is back.
    } else {
        // For store requests, we need to set the data to be sent.
        assert(data_ptr != nullptr);
        pkt->dataStatic<uint8_t>(data_ptr.get());
    }

    AddressTranslationDoneCallbackType done_callback = std::bind(
        &PickleDeviceRequestManager::handleTranslationCompletion,
        this, std::placeholders::_1
    );

    AddressTranslationFaultCallbackType fault_callback = std::bind(
        &PickleDeviceRequestManager::handleTranslationFault,
        this, std::placeholders::_1, std::placeholders::_2
    );

    // Add the request
    outstanding_requests[vaddr].emplace_back(
        new RequestBookkeeper(done_callback, fault_callback, pkt)
    );

    mmu->translateTiming(
        req, owner->getThreadContextPtr(), new PickleDeviceAddressTranslation(
            outstanding_requests[vaddr].back(), requestor_id
        ),
        BaseMMU::Read
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
    bool success = owner->request_port.sendTimingReq(
        request_bookkeeper->getPkt()
    );
    if (success) {
        request_bookkeeper->translationSent();
    } else {
        addRetryHandleTranslationCompletion(request_bookkeeper);
        //DPRINTF(PickleDeviceRequestManagerDebug,
        //    "Retrying translation completion for vaddr 0x%llx\n", vaddr
        //);
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
    }
    //DPRINTF(PickleDeviceRequestManagerDebug,
    //    "Retrying translation completion for vaddr 0x%llx\n", vaddr
    //);
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
            //DPRINTF(PickleDeviceRequestManagerDebug,
            //    "Retrying translation completion for vaddr 0x%llx\n", vaddr
            //);
            break;
        }
        need_retry_handle_translation_completion.pop();
        request_bookkeeper->requestPending();
        //DPRINTF(PickleDeviceRequestManagerDebug,
        //    "Retrying translation completion for vaddr 0x%llx\n", vaddr
        //);
    }
}

void
PickleDeviceRequestManager::handleTranslationFault(
    std::shared_ptr<RequestBookkeeper> request_bookkeeper, const Fault& fault
)
{
    // When the translation fails, we'll notify the requestor.
    // TODO: remove the request from the outstanding requests.
    DPRINTF(PickleDeviceRequestManagerDebug,
        "Translation fault for vaddr 0x%llx\n", request_bookkeeper->getVAddr()
    );
    Addr vaddr = request_bookkeeper->getVAddr();
    if (outstanding_requests.find(vaddr) == outstanding_requests.end()) {
        return;
    }
    // Remove the request from the outstanding requests.
    outstanding_requests[vaddr].erase(
        std::remove_if(
            outstanding_requests[vaddr].begin(),
            outstanding_requests[vaddr].end(),
            [request_bookkeeper](
                const std::shared_ptr<RequestBookkeeper>& rbk
            ) {
                return rbk == request_bookkeeper;
            }
        ),
        outstanding_requests[vaddr].end()
    );
}

}; // namespace gem5
