/*
 * Copyright (c) 2024 Advanced Micro Devices, Inc.
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

#ifndef __TRAFFIC_MUX_HH__
#define __TRAFFIC_MUX_HH__

#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "mem/port.hh"
#include "params/TrafficMux.hh"
#include "sim/clocked_object.hh"
#include "sim/sim_object.hh"

namespace gem5
{

template<typename T>
class UniqueQueue
{
    private:
        std::queue<T> q;
        std::unordered_set<T> m;
    public:
        void push(T val)
        {
            if (m.find(val) == m.end())
            {
                q.push(val);
                m.insert(val);
            }
        }

        T pop()
        {
            T val = q.front();
            q.pop();
            m.erase(val);
            return val;
        }

        T front()
        {
            return q.front();
        }

        bool empty()
        {
            return q.empty();
        }
};

class TrafficMux: public ClockedObject
{
    private:
        class TrafficMuxResponsePort : public ResponsePort
        {
            private:

                /** A reference to the crossbar to which this port belongs. */
                TrafficMux &mux;
                uint64_t internal_id;

            public:

                TrafficMuxResponsePort(
                    const std::string &_name,
                    PortID _id,
                    TrafficMux &_mux,
                    uint64_t _internal_id
                )
                    : ResponsePort(_name, _id), mux(_mux),
                      internal_id(_internal_id)
                { }

            protected:

                AddrRangeList
                getAddrRanges() const override
                {
                    return mux.getAddrRanges();
                }

                Tick
                recvAtomic(PacketPtr pkt) override
                {
                    return mux.recvAtomic(pkt);
                }

                void
                recvFunctional(PacketPtr pkt) override
                {
                    mux.recvFunctional(pkt);
                }

                bool
                recvTimingReq(PacketPtr pkt) override
                {
                    return mux.recvTimingReq(pkt, internal_id);
                }

                void recvRespRetry() override
                {
                    mux.recvRespRetry(id);
                }

        };

        class TrafficMuxRequestPort : public RequestPort
        {
            private:
                /** A reference to the crossbar to which this port belongs. */
                TrafficMux &mux;

            public:

                TrafficMuxRequestPort(
                    const std::string &_name,
                    TrafficMux &_mux
                )
                    : RequestPort(_name), mux(_mux)
                { }

            protected:

                bool
                recvTimingResp(PacketPtr pkt) override
                {
                    return mux.recvTimingResp(pkt);
                }

                void
                recvReqRetry() override
                {
                    mux.recvReqRetry();
                }

                void
                recvRangeChange() override
                {
                    mux.recvRangeChange();
                }
        };

    private:
        TrafficMuxRequestPort mem_side_port;
        std::vector<TrafficMuxResponsePort> cpu_side_ports;
        std::unordered_map<PacketId, uint64_t> where_is_the_port;
        UniqueQueue<uint64_t> retry_queue;
    private:

        // ResponsePort
        AddrRangeList getAddrRanges() const;
        Tick recvAtomic(PacketPtr pkt);
        void recvFunctional(PacketPtr pkt);
        bool recvTimingReq(PacketPtr pkt, uint64_t port_id);
        void recvRespRetry(const PortID id);

        // RequestPort
        bool recvTimingResp(PacketPtr pkt);
        void recvReqRetry();
        void recvRangeChange();
    public:
        TrafficMux(const TrafficMuxParams &params);
        void startup() {};
        Port & getPort(const std::string &if_name, PortID idx) override;
        void switchOn();
        void switchOff();
};

}; // namespace gem5

#endif // __TRAFFIC_MUXER_HH__
