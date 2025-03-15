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
 */

#ifndef PICKLE_ADDRESS_TRANSLATION_HELPER
#define PICKLE_ADDRESS_TRANSLATION_HELPER

#include <functional>
#include <memory>

#include "arch/generic/mmu.hh"
#include "base/trace.hh"
#include "mem/packet.hh"
#include "mem/request.hh"

namespace gem5
{

class RequestBookkeeper;

using AddressTranslationDoneCallbackType = \
    std::function<void(std::shared_ptr<RequestBookkeeper>)>;
using AddressTranslationFaultCallbackType = \
    std::function<void(std::shared_ptr<RequestBookkeeper>, const Fault&)>;

class RequestBookkeeper: public std::enable_shared_from_this<RequestBookkeeper>
{
    private:
        enum RequestStatus
        {
            // the request has been queued but nothing has been done
            TRANSLATION_PENDING = 0,
            // the address translation request has been sent
            TRANSLATION_SENT = 1,
            // the address translation for the request is done, but the
            // request has not been sent to the cache hierarchy
            REQUEST_PENDING = 2,
            // the request has been sent to the cache hierarchy
            REQUEST_SENT = 3,
            // the response is ready
            RESPONSE_READY = 4
        };
        AddressTranslationDoneCallbackType done_callback;
        AddressTranslationFaultCallbackType fault_callback;
        PacketPtr pkt;
        RequestPtr req;
        Fault fault;
        Addr vaddr;
        Addr paddr;
        RequestStatus status;
    public:
        RequestBookkeeper(
            AddressTranslationDoneCallbackType _done_callback,
            AddressTranslationFaultCallbackType _fault_callback,
            PacketPtr _pkt
        ) : done_callback(_done_callback), fault_callback(_fault_callback),
            pkt(_pkt), req(_pkt->req), fault(NoFault), paddr(0xBADADD),
            status(RequestStatus::TRANSLATION_PENDING)
        {}
        void translationSent() { status = RequestStatus::TRANSLATION_SENT; }
        void requestPending() { status = RequestStatus::REQUEST_PENDING; }
        void requestSent() { status = RequestStatus::REQUEST_SENT; }
        void respondReady() { status = RequestStatus::RESPONSE_READY; }
        void setResult(const Fault& _fault, const RequestPtr &req)
        {
            status = RequestStatus::REQUEST_PENDING;
            fault = _fault;
            if (fault == NoFault) {
                paddr = req->getPaddr();
                done_callback(shared_from_this());
            } else {
                fault_callback(shared_from_this(), fault);
            }
        }
        PacketPtr getPkt() { return pkt; }
        Addr getVAddr() const { return req->getVaddr(); }
};

class PickleDeviceAddressTranslation : public BaseMMU::Translation
{
    private:
    std::shared_ptr<RequestBookkeeper> bookkeeper;
        RequestPtr req;
    public:
        PickleDeviceAddressTranslation(
            std::shared_ptr<RequestBookkeeper> _bookkeeper,
            const RequestorID& requestor_id
        ) : bookkeeper(_bookkeeper)
        {
            Request::Flags flags;
            Addr vaddr = bookkeeper->getVAddr();
            req = std::make_shared<Request>(
                vaddr, 64, flags, requestor_id, 0, 0
            );
        }

        ~PickleDeviceAddressTranslation() {
        }

        RequestPtr getRequest() { return req; }

        void markDelayed() override {}

        void finish(
            const Fault &fault, const RequestPtr &req, ThreadContext *tc,
            BaseMMU::Mode mode
        ) override {
            bookkeeper->setResult(fault, req);
            delete this;
        }
};

}; // namespace gem5

#endif // PICKLE_ADDRESS_TRANSLATION_HELPER
