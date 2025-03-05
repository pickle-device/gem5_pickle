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

#ifndef __PICKLE_DEVICE_HH__
#define __PICKLE_DEVICE_HH__

#include <queue>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "base/logging.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "mem/ruby/protocol/CHI/Cache_Controller.hh"
#include "params/PickleDevice.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"
#include "sim/sim_object.hh"

namespace gem5
{

enum PickleDeviceState
{
    SLEEP = 0,
    IDLE = 1,
    RECEIVING_COMMAND = 2
};

enum PickleDeviceCommandType
{
    INVALID = 0,
    ADD_WATCH_RANGE = 1,
    JOB_DESCRIPTOR = 2
};

class PickleDevice: public ClockedObject
{
    public:
        PickleDevice(const PickleDeviceParams &params);
        ~PickleDevice();
        void startup() override;
    public:
        void switchOn();
        void switchOff();
        void wakeup();
        void operate();
        void operateResponseQueue();
        void processEvent(); // what to do per event
    private:
        PARAMS(PickleDevice);
        EventFunctionWrapper event;
        System * system;
        uint64_t num_cores;
        uint64_t device_id;
        uint64_t core_to_pickle_latency_in_ticks;
        uint64_t ticks_per_cycle;
        PickleDeviceState device_state;
    public:
        // this is a request port from the engine to its controller
        // this port allows the engine to make requests to controller as if
        // the engine is a CPU
        class PickleDeviceRequestPort: public RequestPort
        {
            private:
                PickleDevice* owner;
            public:
                PickleDeviceRequestPort(
                    const std::string& name, PickleDevice* owner
                ) : RequestPort(name)
                {
                    this->owner = owner;
                }
                ~PickleDeviceRequestPort() {}
                bool recvTimingResp(PacketPtr pkt) override;
                void recvReqRetry() override;
        };
        class PickleDeviceUncacheableSnoopPort: public ResponsePort
        {
            private:
                PickleDevice* owner;
                uint8_t internal_id;
            public:
                PickleDeviceUncacheableSnoopPort(
                    const std::string& name, PickleDevice* owner, PortID id,
                    uint8_t internal_id
                ) : ResponsePort(name, id)
                {
                    this->owner = owner;
                    this->internal_id = internal_id;
                }
                ~PickleDeviceUncacheableSnoopPort() {}
                // will be called when CPU sends uncacheable loads/stores
                bool recvTimingReq(PacketPtr pkt) override;
                void recvRespRetry() override {};
                // found return 0 in many existing codes
                Tick recvAtomic(PacketPtr pkt) override { return 0;}
                void recvFunctional(PacketPtr pkt) override {};
                AddrRangeList getAddrRanges() const override {
                    AddrRangeList ranges;
                    ranges.push_back(AddrRange(0,MaxAddr));
                    return ranges;
                }
        };
        PickleDeviceRequestPort request_port;
        std::vector<PickleDeviceUncacheableSnoopPort*> uncacheable_snoop_ports;
    public:
        RequestorID requestor_id;
    private:
        void doIdle();
        void doReceivingCommand();
        void doBusy();
        void changeToState(const PickleDeviceState new_state);
        Port& getPort(const std::string &if_name, PortID idx) override;
        void addWatchRange(AddrRange r);
    private:
        uint64_t remaining_control_message_length;
        uint64_t remaining_control_data_length;
        PickleDeviceCommandType receiving_command_type;
        std::vector<uint8_t> received_control_message;
        std::vector<uint8_t> received_control_data;
        std::vector<AddrRange> watch_ranges;
        std::vector<std::queue<PacketPtr>> uncacheable_response_queues;
        uint64_t uncacheable_response_queue_capacity;
        uint64_t response_queue_progress_per_cycle;
        void processJobDescriptor(std::vector<uint8_t>& job_descriptor);
    public:
        void enqueueControlMessage(uint8_t message);
        void enqueueControlData(uint8_t data);
        bool enqueueResponse(PacketPtr pkt, uint8_t internal_port_id);
    public:
        struct PickleDeviceStats : public statistics::Group
        {
            PickleDeviceStats(statistics::Group *parent);
            void regStats() override;
            statistics::Scalar numTranslationFaults;
        } device_stats;
        void regStats() override;
}; // class PickleDevice

}; // namespace gem5

#endif // __PICKLE_DEVICE_HH__
