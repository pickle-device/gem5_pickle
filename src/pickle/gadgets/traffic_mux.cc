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

#include "pickle/gadgets/traffic_mux.hh"

#include "debug/TrafficMuxDebug.hh"

namespace gem5
{

TrafficMux::TrafficMux(const TrafficMuxParams &params)
  : ClockedObject(params),
    mem_side_port(params.name + ".mem_side_port", *this)
{
     for (int i = 0; i < params.port_rsp_ports_connection_count; ++i) {
        cpu_side_ports.emplace_back(
            name() + csprintf(".cpu_side_ports[%d]", i), i, *this, i
        );
    }
}

// ResponsePort

AddrRangeList
TrafficMux::getAddrRanges() const
{
    return mem_side_port.getAddrRanges();
}

Tick
TrafficMux::recvAtomic(PacketPtr pkt)
{
    return mem_side_port.sendAtomic(pkt);
}

void
TrafficMux::recvFunctional(PacketPtr pkt)
{
    mem_side_port.sendFunctional(pkt);
}

bool
TrafficMux::recvTimingReq(PacketPtr pkt, uint64_t internal_id)
{
    if (mem_side_port.sendTimingReq(pkt)) {
        where_is_the_port[pkt->id] = internal_id;
        return true;
    }

    DPRINTF(
        TrafficMuxDebug,
        "Failed to send timing request on port %lu\n",
        internal_id
    );
    retry_queue.push(internal_id);
    return false;
}

bool
TrafficMux::recvTimingResp(PacketPtr pkt)
{
    PacketId pkt_id = pkt->id;
    return cpu_side_ports[where_is_the_port[pkt_id]].sendTimingResp(pkt);
}

void
TrafficMux::recvRangeChange()
{
    for (auto p : cpu_side_ports)
        p.sendRangeChange();
}

Port &
TrafficMux::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "req_port") {
      return mem_side_port;
    } else if (if_name == "rsp_ports" && idx < cpu_side_ports.size()) {
        // We should have already created all of the ports in the constructor
        return cpu_side_ports[idx];
    }
    return ClockedObject::getPort(if_name, idx);
}

void
TrafficMux::recvRespRetry(const PortID id)
{
    mem_side_port.sendRetryResp();
    DPRINTF(TrafficMuxDebug, "Received response retry: id=%ld\n", id);
};

void
TrafficMux::recvReqRetry()
{
    while (!retry_queue.empty()) {
        uint64_t id = retry_queue.front();
        cpu_side_ports[id].sendRetryReq();
        retry_queue.pop();
        DPRINTF(TrafficMuxDebug, "Retrying request on port: %lu\n", id);
    }
};

}; // namespace gem5
