/*
 * Copyright (c) 2024 Advanced Micro Devices, Inc.
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

#ifndef __TRAFFIC_SNOOPER_HH__
#define __TRAFFIC_SNOOPER_HH__

#include <vector>

#include "mem/port.hh"
#include "params/TrafficSnooper.hh"
#include "sim/clocked_object.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class TrafficSnooper: public ClockedObject
{
    private:
        class TrafficSnooperResponsePort : public ResponsePort
        {
            private:

                /** A reference to the crossbar to which this port belongs. */
                TrafficSnooper &snooper;

            public:

                TrafficSnooperResponsePort(
                    const std::string &_name,
                    TrafficSnooper &_snooper,
                    PortID _id
                )
                    : ResponsePort(_name, _id), snooper(_snooper)
                { }

            protected:

                bool
                recvTimingReq(PacketPtr pkt) override
                {
                    return snooper.recvTimingReq(pkt);
                }

                bool
                recvTimingSnoopResp(PacketPtr pkt) override
                {
                    return snooper.recvTimingSnoopResp(pkt);
                }

                Tick
                recvAtomic(PacketPtr pkt) override
                {
                    return snooper.recvAtomic(pkt);
                }

                Tick
                recvAtomicBackdoor(PacketPtr pkt, MemBackdoorPtr &backdoor)
                override
                {
                    return snooper.recvAtomicBackdoor(pkt, backdoor);
                }

                void
                recvFunctional(PacketPtr pkt) override
                {
                    snooper.recvFunctional(pkt);
                }

                void
                recvMemBackdoorReq(const MemBackdoorReq &req,
                        MemBackdoorPtr &backdoor) override
                {
                    snooper.recvMemBackdoorReq(req, backdoor);
                }

                AddrRangeList
                getAddrRanges() const override
                {
                    return snooper.getAddrRanges();
                }

                void recvRespRetry() override
                {
                    snooper.recvRespRetry();
                }

        };

        class TrafficSnooperRequestPort : public RequestPort
        {
            private:
                /** A reference to the crossbar to which this port belongs. */
                TrafficSnooper &snooper;

            public:

                TrafficSnooperRequestPort(
                    const std::string &_name,
                    TrafficSnooper &_snooper,
                    PortID _id
                )
                    : RequestPort(_name, _id), snooper(_snooper)
                { }

            protected:

                /**
                * Determine if this port should be considered a snooper. For
                * a coherent crossbar memory-side port this is always true.
                *
                * @return a boolean that is true if this port is snooping
                */
                bool isSnooping() const override { return true; }

                bool
                recvTimingResp(PacketPtr pkt) override
                {
                    return snooper.recvTimingResp(pkt);
                }

                void
                recvTimingSnoopReq(PacketPtr pkt) override
                {
                    return snooper.recvTimingSnoopReq(pkt);
                }

                Tick
                recvAtomicSnoop(PacketPtr pkt) override
                {
                    return snooper.recvAtomicSnoop(pkt);
                }

                void
                recvFunctionalSnoop(PacketPtr pkt) override
                {
                    snooper.recvFunctionalSnoop(pkt);
                }

                void
                recvRangeChange() override
                {
                    snooper.recvRangeChange();
                }
                void
                recvReqRetry() override
                {
                    snooper.recvReqRetry();
                }
        };

    private:
        ResponsePort* in_port;
        RequestPort* out_port;
        RequestPort* snoop_port;
        bool isActivated;
        std::vector<AddrRange> watch_ranges;
    private:
        // ResponsePort
        bool recvTimingReq(PacketPtr pkt);
        bool recvTimingSnoopResp(PacketPtr pkt);
        Tick recvAtomic(PacketPtr pkt);
        Tick recvAtomicBackdoor(PacketPtr pkt, MemBackdoorPtr &backdoor);
        void recvFunctional(PacketPtr pkt);
        void recvMemBackdoorReq(
            const MemBackdoorReq &req, MemBackdoorPtr &backdoor);
        AddrRangeList getAddrRanges() const;
        void recvRespRetry();
        // RequestPort
        bool recvTimingResp(PacketPtr pkt);
        void recvTimingSnoopReq(PacketPtr pkt);
        Tick recvAtomicSnoop(PacketPtr pkt);
        void recvFunctionalSnoop(PacketPtr pkt);
        void recvRangeChange();
        void recvReqRetry();
        // utilities
        PacketPtr duplicatePkt(PacketPtr pkt) const;
    public:
        TrafficSnooper(const TrafficSnooperParams &params);
        ~TrafficSnooper();
        void startup();
        Port & getPort(const std::string &if_name, PortID idx) override;
        void switchOn();
        void switchOff();
        void addWatchRange(AddrRange range);
        bool inRange(Addr addr);
};

}; // namespace gem5

#endif // __TRAFFIC_SNOOPER_HH__
