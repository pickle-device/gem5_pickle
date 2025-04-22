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

#include "pickle/device/pickle_device.hh"

#include "arch/arm/regs/misc.hh"
#include "base/trace.hh"
#include "debug/PickleDeviceAddressTranslation.hh"
#include "debug/PickleDeviceControl.hh"
#include "debug/PickleDeviceEvent.hh"
#include "debug/PickleDeviceState.hh"
#include "debug/PickleDeviceUncacheableForwarding.hh"
#include "debug/PickleDeviceZeroCycleAddressTranslation.hh"
#include "mem/request.hh"
#include "mem/ruby/system/RubySystem.hh"
#include "pickle/application_specific/pickle_job.hh"
#include "pickle/request_manager/manager.hh"

namespace gem5
{

PickleDevice::PickleDevice(const PickleDeviceParams& params)
  : ClockedObject(params),
    event([this]{processEvent();}, name() + ".event"),
    operate_uncacheable_response_queue_event(
        [this]{operateUncacheableResponseQueue();},
        name() + ".operate_uncacheable_response_queue_event"
    ),
    system(params.system),
    //mmu(params.mmu),
    //isa(params.isa),
    decoder(params.decoder),
    associated_cores(params.associated_cores),
    num_cores(params.num_cores),
    device_id(params.device_id),
    core_to_pickle_latency_in_ticks(params.core_to_pickle_latency_in_ticks),
    ticks_per_cycle(params.ticks_per_cycle),
    //device_state(PickleDeviceState::SLEEP),
    request_port(params.name + ".request_port", this),
    uncacheable_forwarders(params.uncacheable_forwarders),
    request_manager(params.request_manager),
    remaining_control_message_length(0),
    remaining_control_data_length(0),
    receiving_command_type(PickleDeviceCommandType::INVALID),
    uncacheable_response_queues(num_cores),
    uncacheable_response_queue_capacity(
        params.uncacheable_response_queue_capacity
    ),
    response_queue_progress_per_cycle(
        params.response_queue_progress_per_cycle
    ),
    device_thread_context(nullptr),
    device_command_address(0),
    coalesce_requests(params.coalesce_requests),
    coalesce_address_translations(params.coalesce_address_translations),
    pickle_prefetcher(params.prefetcher),
    device_stats(this)
{
    if (params.is_on) {
        changeToState(PickleDeviceState::IDLE);
    } else {
        changeToState(PickleDeviceState::SLEEP);
    }
    mmu = dynamic_cast<ArmISA::MMU*>(params.mmu);
    isa = dynamic_cast<ArmISA::ISA*>(params.isa);

    requestor_id = system->getRequestorId(this);

    for (int i = 0; i < num_cores; ++i) {
        uncacheable_snoop_ports.push_back(
            new PickleDeviceUncacheableSnoopPort(
                csprintf("%s%d", "uncacheable_snoop_port", i), this, i, i));
    }
}

PickleDevice::~PickleDevice()
{
}

void
PickleDevice::startup()
{
    request_manager->setOwner(this);
    request_manager->setMMU(mmu);
    request_manager->setRequestorID(requestor_id);
    pickle_prefetcher->setOwner(this);
}

void
PickleDevice::switchOn()
{
    schedule(event, curTick() + 2);
    DPRINTF(
        PickleDeviceEvent, "Switching on: schedule event at %lld\n",
        curTick() + 2
    );
    changeToState(PickleDeviceState::IDLE);
}

void
PickleDevice::switchOff()
{
    changeToState(PickleDeviceState::SLEEP);
}

void
PickleDevice::wakeup()
{
    processEvent();
}

void
PickleDevice::processEvent()
{
    switch (this->device_state)
    {
        case PickleDeviceState::IDLE:
            this->doIdle();
            this->operate();
            break;
        case PickleDeviceState::RECEIVING_COMMAND:
            this->operate();
            break;
        case PickleDeviceState::SLEEP:
        default:
            break;
    }
}

void
PickleDevice::operate()
{
    DPRINTF(
        PickleDeviceEvent, "Operate: schedule event at %lld\n",
        curTick() + ticks_per_cycle
    );
}

void
PickleDevice::operateUncacheableResponseQueue()
{
    bool has_remaining_packets = false;
    for (
        uint8_t port_id = 0;
        port_id < uncacheable_response_queues.size();
        port_id++
    ) {
        uint64_t remain_progress = response_queue_progress_per_cycle;
        if (response_queue_progress_per_cycle == 0) {
            remain_progress = -1ULL;
        }
        while (   (remain_progress > 0)
               && (uncacheable_response_queues[port_id].size() > 0)) {
            PacketPtr front_pkt = uncacheable_response_queues[port_id].front();
            assert(front_pkt->isResponse());
            bool success = uncacheable_snoop_ports[port_id]->sendTimingResp(
                front_pkt
            );
            if (success) {
                uncacheable_response_queues[port_id].pop();
                remain_progress -= 1;
            } else {
                break;
            }
        }
        if (uncacheable_response_queues[port_id].size() > 0) {
            has_remaining_packets = true;
        }
    }
    // schedule the next event if there are still packets in the queues
    if (has_remaining_packets) {
        scheduleOperateUncacheableResponseQueueEvent();
    }
}


void
PickleDevice::doIdle()
{
    // TODO
}

void
PickleDevice::changeToState(const PickleDeviceState new_state)
{
    this->device_state = new_state;
    DPRINTF(
        PickleDeviceState, "State changed to %d\n", new_state
    );
}

Port&
PickleDevice::getPort(const std::string &if_name, PortID idx)
{
    DPRINTF(
        PickleDeviceState, "Get port %s\n", if_name
    );
    if (if_name == "request_port") {
        return request_port;
    } else if (if_name == "uncacheable_snoop_port") {
        return *uncacheable_snoop_ports[idx];
    }
    return request_port;
}

PickleDevice::PickleDeviceStats::PickleDeviceStats(statistics::Group *parent)
    : statistics::Group(parent),
      ADD_STAT(
        numTranslationFaults,
        "The number of faults produced by address translations"
      )
{
}

void
PickleDevice::PickleDeviceStats::regStats()
{
}

void
PickleDevice::regStats()
{
    ClockedObject::regStats();
    device_stats.regStats();
}

bool
PickleDevice::PickleDeviceRequestPort::recvTimingResp(PacketPtr pkt)
{
    auto req = pkt->req;
    DPRINTF(PickleDeviceRequestManagerDebug,
        "Received response: vaddr = 0x%llx, paddr = 0x%llx\n",
        req->getVaddr(), req->getPaddr()
    );
    const uint8_t* pkt_data = pkt->getConstPtr<uint8_t>();
    uint8_t* data = new uint8_t[req->getSize()];
    std::memcpy(data, pkt_data, req->getSize());
    owner->pickle_prefetcher->receivePrefetch(
        req->getVaddr(), std::unique_ptr<uint8_t[]>(data)
    );
    owner->request_manager->handleRequestCompletion(pkt);
    return true;
}

void
PickleDevice::PickleDeviceRequestPort::recvReqRetry()
{
    // we don't need to do anything, we will try to send request out again
    // in the following cycles
}

// only uncacheable request will be forward here from the snooper
bool
PickleDevice::PickleDeviceUncacheableSnoopPort::recvTimingReq(PacketPtr pkt)
{
    // handle when the engine receives an uncacheable load/store
    {
        // 0x10110000 sends:
        //   uint64_t: command type
        //   uint64_t: command length
        if (pkt->req->hasPaddr() && pkt->req->getPaddr() == 0x10110000) {
            const uint8_t* ptr = pkt->getConstPtr<uint8_t>();
            uint8_t data = ptr[0];
            owner->enqueueControlMessage(data);
            DPRINTF(
                PickleDeviceUncacheableForwarding,
                "Received control message: addr = 0x%llx, data = 0x%x\n",
                pkt->req->getPaddr(), data
            );
            if (pkt->needsResponse()) {
                pkt->makeResponse();
                bool success = owner->enqueueResponse(pkt, internal_id);
                assert(success);
            }
        }
        // 0x10110008 sends data accordingly to command type and length
        else if (pkt->req->hasPaddr() && pkt->req->getPaddr() == 0x10110008) {
            const uint8_t* ptr = pkt->getConstPtr<uint8_t>();
            uint8_t data = ptr[0];
            owner->enqueueControlData(data);
            DPRINTF(
                PickleDeviceUncacheableForwarding,
                "Received control data: addr = 0x%llx, data = 0x%x\n",
                pkt->req->getPaddr(), data
            );
            if (pkt->needsResponse()) {
                pkt->makeResponse();
                bool success = owner->enqueueResponse(pkt, internal_id);
                assert(success);
            }
        } // device specs
        else if (
            pkt->req->hasPaddr()
                && (pkt->req->getPaddr() >= 0x10110010
                    && pkt->req->getPaddr() <= 0x10110020)
        ) {
            uint64_t data = -1;
            Addr paddr = pkt->req->getPaddr();
            switch (paddr) {
                case 0x10110010:
                    data = owner->getDeviceState() != PickleDeviceState::SLEEP;
                    break;
                case 0x10110018:
                    data = owner->getPrefetcher()\
                                ->getSoftwareHintPrefetchDistance();
                    break;
            };
            uint8_t* data_ptr = (uint8_t*) &data;
            pkt->makeResponse();
            pkt->setData(data_ptr);
            DPRINTF(
                PickleDeviceControl,
                "Sent to paddr: 0x%llx, size: %ld, data: 0x%ld\n",
                paddr, pkt->req->getSize(), data
            );
            bool success = owner->enqueueResponse(pkt, internal_id);
            assert(success);
        } else {
            owner->trySetThreadContextFromCore(internal_id);
            bool isLoad = pkt->isRead();
            if (isLoad) {
                DPRINTF(
                    PickleDeviceUncacheableForwarding,
                    "Received load request: addr = 0x%llx\n",
                    pkt->req->getPaddr()
                );
            } else {
                const uint64_t* ptr = pkt->getConstPtr<uint64_t>();
                uint64_t data = ptr[0];
                owner->pickle_prefetcher->enqueueWork(data, internal_id);
                DPRINTF(
                    PickleDeviceUncacheableForwarding,
                    "Received store request: addr = 0x%llx, data = 0x%llx\n",
                    pkt->req->getPaddr(), data
                );
            }
            if (pkt->needsResponse()) {
                pkt->makeResponse();
                bool success = owner->enqueueResponse(pkt, internal_id);
                assert(success);
            }
        }
    }

    return true;
}

Tick
PickleDevice::PickleDeviceUncacheableSnoopPort::recvAtomic(PacketPtr pkt)
{
    {
        // 0x10110000 sends:
        //   uint64_t: command type
        //   uint64_t: command length
        if (pkt->req->hasPaddr() && pkt->req->getPaddr() == 0x10110000) {
            const uint8_t* ptr = pkt->getConstPtr<uint8_t>();
            uint8_t data = ptr[0];
            owner->enqueueControlMessage(data);
            DPRINTF(
                PickleDeviceUncacheableForwarding,
                "Received control message [atomic]: "
                "addr = 0x%llx, data = 0x%x\n",
                pkt->req->getPaddr(), data
            );
        }
        // 0x10110008 sends data accordingly to command type and length
        else if (pkt->req->hasPaddr() && pkt->req->getPaddr() == 0x10110008) {
            const uint8_t* ptr = pkt->getConstPtr<uint8_t>();
            uint8_t data = ptr[0];
            owner->enqueueControlData(data);
            DPRINTF(
                PickleDeviceUncacheableForwarding,
                "Received control data [atomic]: "
                "addr = 0x%llx, data = 0x%x\n",
                pkt->req->getPaddr(), data
            );
        } // device specs
        else if (
            pkt->req->hasPaddr()
                && (pkt->req->getPaddr() >= 0x10110010
                    && pkt->req->getPaddr() <= 0x10110020)
        ) {
            uint64_t data = -1;
            Addr paddr = pkt->req->getPaddr();
            switch (paddr) {
                case 0x10110010:
                    data = owner->getDeviceState() != PickleDeviceState::SLEEP;
                    break;
                case 0x10110018:
                    data = owner->getPrefetcher()\
                                ->getSoftwareHintPrefetchDistance();
                    break;
            };
            uint8_t* data_ptr = (uint8_t*) &data;
            pkt->makeResponse();
            pkt->setData(data_ptr);
            DPRINTF(
                PickleDeviceControl,
                "Sent to paddr: 0x%llx, size: %ld, data: 0x%ld\n",
                paddr, pkt->req->getSize(), data
            );
        } else {
            owner->trySetThreadContextFromCore(internal_id);
            bool isLoad = pkt->isRead();
            std::string type = isLoad ? "Load" : "Store";
            DPRINTF(
                PickleDeviceUncacheableForwarding,
                "Received request [atomic]: addr = 0x%llx, type = %s\n",
                pkt->req->getPaddr(), type.c_str()
            );
        }
    }
    return 0;
}

void
PickleDevice::PickleDeviceUncacheableSnoopPort::recvFunctional(PacketPtr pkt)
{
    {
        // 0x10110000 sends:
        //   uint64_t: command type
        //   uint64_t: command length
        if (pkt->req->hasPaddr() && pkt->req->getPaddr() == 0x10110000) {
            const uint8_t* ptr = pkt->getConstPtr<uint8_t>();
            uint8_t data = ptr[0];
            owner->enqueueControlMessage(data);
            DPRINTF(
                PickleDeviceUncacheableForwarding,
                "Received control message [functional]: "
                "addr = 0x%llx, data = 0x%x\n",
                pkt->req->getPaddr(), data
            );
        }
        // 0x10110008 sends data accordingly to command type and length
        else if (pkt->req->hasPaddr() && pkt->req->getPaddr() == 0x10110008) {
            const uint8_t* ptr = pkt->getConstPtr<uint8_t>();
            uint8_t data = ptr[0];
            owner->enqueueControlData(data);
            DPRINTF(
                PickleDeviceUncacheableForwarding,
                "Received control data [functional]: "
                "addr = 0x%llx, data = 0x%x\n",
                pkt->req->getPaddr(), data
            );
        } // device specs
        else if (
            pkt->req->hasPaddr()
                && (pkt->req->getPaddr() >= 0x10110010
                    && pkt->req->getPaddr() <= 0x10110020)
        ) {
            uint64_t data = -1;
            Addr paddr = pkt->req->getPaddr();
            switch (paddr) {
                case 0x10110010:
                    data = owner->getDeviceState() != PickleDeviceState::SLEEP;
                    break;
                case 0x10110018:
                    data = owner->getPrefetcher()\
                                ->getSoftwareHintPrefetchDistance();
                    break;
            };
            uint8_t* data_ptr = (uint8_t*) &data;
            pkt->makeResponse();
            pkt->setData(data_ptr);
            DPRINTF(
                PickleDeviceControl,
                "Sent to paddr: 0x%llx, size: %ld, data: 0x%ld\n",
                paddr, pkt->req->getSize(), data
            );
        } else {
            owner->trySetThreadContextFromCore(internal_id);
            bool isLoad = pkt->isRead();
            if (isLoad) {
                DPRINTF(
                    PickleDeviceUncacheableForwarding,
                    "Received load request [functional]: "
                    "addr = 0x%llx\n",
                    pkt->req->getPaddr()
                );
            } else {
                const uint64_t* ptr = pkt->getConstPtr<uint64_t>();
                uint64_t data = ptr[0];
                DPRINTF(
                    PickleDeviceUncacheableForwarding,
                    "Received store request [functional]: "
                    "addr = 0x%llx, data = 0x%llx\n",
                    pkt->req->getPaddr(), data
                );
            }
        }
    }
}

void
PickleDevice::enqueueControlMessage(uint8_t message)
{
    if (device_state == PickleDeviceState::IDLE) {
        remaining_control_message_length = 16;
        changeToState(PickleDeviceState::RECEIVING_COMMAND);
        DPRINTF(
            PickleDeviceControl,
            "Start receiving command\n"
        );
    }
    remaining_control_message_length -= 1;
    received_control_message.push_back(message);
    if (remaining_control_message_length == 0) {
        uint8_t* ptr8 = received_control_message.data();
        uint64_t* ptr64 = (uint64_t*) ptr8;
        receiving_command_type = PickleDeviceCommandType(ptr64[0]);
        remaining_control_data_length = ptr64[1];
        remaining_control_message_length = 0;
        received_control_message.clear();
        DPRINTF(
            PickleDeviceControl,
            "Received command type 0x%llx, length %lld\n",
            receiving_command_type, remaining_control_data_length
        );
    }
}

void
PickleDevice::enqueueControlData(uint8_t data)
{
    remaining_control_data_length -= 1;
    received_control_data.push_back(data);
    if (remaining_control_data_length == 0) {
        if (
            receiving_command_type == PickleDeviceCommandType::ADD_WATCH_RANGE
        ) {
            uint8_t* ptr8 = received_control_data.data();
            uint64_t* ptr64 = (uint64_t*) ptr8;
            addWatchRange(AddrRange(ptr64[0], ptr64[1]));
            device_command_address = ptr64[0];
            DPRINTF(
                PickleDeviceControl,
                "Added watch range 0x%llx - 0x%llx\n",
                ptr64[0], ptr64[1]
            );
        } else if (
            receiving_command_type == PickleDeviceCommandType::JOB_DESCRIPTOR
        ) {
            processJobDescriptor(received_control_data);
        } else {
            panic("Unknown Command Type %lld\n", receiving_command_type);
        }
        received_control_data.clear();
        receiving_command_type = PickleDeviceCommandType::INVALID;
        changeToState(PickleDeviceState::IDLE);
    }
    DPRINTF(
        PickleDeviceControl,
        "Received command data 0x%x, remaining %lld\n",
        data, remaining_control_data_length
    );
}

bool
PickleDevice::enqueueResponse(PacketPtr pkt, uint8_t internal_port_id)
{
    if ((uncacheable_response_queue_capacity > 0) &&
        (uncacheable_response_queues[internal_port_id].size()
            >= uncacheable_response_queue_capacity))
    {
        return false;
    }
    uncacheable_response_queues[internal_port_id].push(pkt);
    scheduleOperateUncacheableResponseQueueEvent();
    return true;
}

void
PickleDevice::handleRequestTranslationFault(const Addr vaddr)
{
    pickle_prefetcher->receivePrefetch(vaddr, nullptr);
}

void
PickleDevice::addWatchRange(AddrRange r)
{
    DPRINTF(
        PickleDeviceControl,
        "Boardcasting new range 0x%llx - 0x%llx\n",
        r.start(), r.end()
    );
    for (auto forwarder: uncacheable_forwarders)
    {
        DPRINTF(
            PickleDeviceControl,
            " -> to %s\n",
            forwarder->name()
        );
        forwarder->addWatchRange(r);
    }
}

void
PickleDevice::scheduleOperateUncacheableResponseQueueEvent()
{
    if (!operate_uncacheable_response_queue_event.scheduled()) {
        schedule(
            operate_uncacheable_response_queue_event,
            curTick() + ticks_per_cycle
        );
    }
}

void
PickleDevice::processJobDescriptor(std::vector<uint8_t>& _job_descriptor)
{
    job_descriptor = std::shared_ptr<PickleJobDescriptor>(
        new PickleJobDescriptor(_job_descriptor)
    );
    pickle_prefetcher->configure(job_descriptor);
    DPRINTF(
        PickleDeviceControl,
        "Received job descriptor: %s\n",
        job_descriptor->to_string()
    );
}

void
PickleDevice::trySetThreadContextFromCore(uint64_t core_id)
{
    if (device_thread_context == nullptr) {
        device_thread_context = std::unique_ptr<PickleDeviceThreadContext>(
            new PickleDeviceThreadContext(this)
        );
        isa->setThreadContext(device_thread_context.get());
        device_thread_context->copyState(
            associated_cores[core_id]->getContext(0)
        );
        // MMU::takeOverFrom
        auto *other_mmu = dynamic_cast<ArmISA::MMU*>(
            associated_cores[core_id]->getContext(0)->getMMUPtr()
        );
        mmu->s1State = other_mmu->s1State;
        mmu->s2State = other_mmu->s2State;
        mmu->_attr = other_mmu->_attr;
        DPRINTF(
            PickleDeviceAddressTranslation,
            "core thread_id: 0x%llx, device thread_id: 0x%llx\n",
            associated_cores[core_id]->getContext(0)->contextId(),
            device_thread_context->contextId()
        );
        // ISA::takeOverFrom
        isa->takeOverFrom(device_thread_context.get(), nullptr);
        DPRINTF(
            PickleDeviceAddressTranslation,
            "Copy the arch regs to device_thread_context from core %lld\n",
            core_id
        );
        DPRINTF(
            PickleDeviceAddressTranslation,
            "TTBR0_EL1: 0x%llx\n",
            device_thread_context->readMiscReg(ArmISA::MISCREG_TTBR0_EL1)
        );
    }
}

uint64_t
PickleDevice::getNumTicksPerCycle() const
{
    return ticks_per_cycle;
}

System *
PickleDevice::getSystem()
{
    return system;
}

BaseMMU *
PickleDevice::getMMUPtr()
{
    return mmu;
}

BaseISA *
PickleDevice::getIsaPtr()
{
    return isa;
}

InstDecoder *
PickleDevice::getDecoderPtr()
{
    return decoder;
}

ThreadContext *
PickleDevice::getThreadContextPtr()
{
    return device_thread_context.get();
}

PickleDeviceState
PickleDevice::getDeviceState() const
{
    return device_state;
}

PicklePrefetcher *
PickleDevice::getPrefetcher()
{
    return pickle_prefetcher;
}

uint64_t
PickleDevice::getNumCores() const
{
    return num_cores;
}

Addr
PickleDevice::getCommandAddr() const
{
    return device_command_address;
}

std::shared_ptr<PickleJobDescriptor>
PickleDevice::getJobDescriptor() const
{
    return job_descriptor;
}

PacketPtr
PickleDevice::zeroCycleLoadWithPAddr(const Addr& paddr, bool& success)
{
    const Addr BLOCK_SIZE = 64;
    Request::Flags flags = 0;
    MemCmd cmd = MemCmd::ReadReq;
    RequestPtr req = std::make_shared<Request>(
        (paddr >> 6) << 6, BLOCK_SIZE, flags, requestor_id);
    uint8_t* data = new uint8_t[BLOCK_SIZE];
    PacketPtr pkt = new Packet(req, cmd, BLOCK_SIZE);
    pkt->dataDynamic<uint8_t>(data);
    request_port.sendFunctional(pkt);
    success = true;
    return pkt;
}

PacketPtr
PickleDevice::zeroCycleLoadWithVAddr(const Addr& vaddr, bool& success)
{
    const Addr BLOCK_SIZE = 64;
    Request::Flags flags = 0;
    MemCmd cmd = MemCmd::ReadReq;
    RequestPtr req1 = std::shared_ptr<Request>(
        new Request(vaddr, 64, Request::Flags(0), this->requestor_id, -1, 0)
    );
    Fault f = mmu->translateFunctional(
        req1, device_thread_context.get(), BaseMMU::Mode::Read
    );
    Addr paddr = req1->getPaddr();
    RequestPtr req = std::make_shared<Request>(
        (paddr >> 6) << 6, BLOCK_SIZE, flags, requestor_id);
    uint8_t* data = new uint8_t[BLOCK_SIZE];
    PacketPtr pkt = new Packet(req, cmd, BLOCK_SIZE);
    pkt->dataDynamic<uint8_t>(data);
    request_port.sendFunctional(pkt);
    success = true;
    return pkt;
}

}; // namespace gem5
