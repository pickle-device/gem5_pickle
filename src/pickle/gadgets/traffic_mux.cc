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
    reqPort(params.name + ".reqPort", *this),
    isActivated(false),
    event([this]{processRetry();}, name() + ".event")
{
                                     // python port name here
     for (int i = 0; i < params.port_rsp_ports_connection_count; ++i) {
        rspPorts.emplace_back(name() + csprintf(".rspPorts[%d]", i), i, *this);
    }
}

// ResponsePort

AddrRangeList
TrafficMux::getAddrRanges() const
{
    return reqPort.getAddrRanges();
}

Tick
TrafficMux::recvAtomic(PacketPtr pkt)
{
    return reqPort.sendAtomic(pkt);
}

void
TrafficMux::recvFunctional(PacketPtr pkt)
{
    reqPort.sendFunctional(pkt);
}

bool
TrafficMux::recvTimingReq(PacketPtr pkt, std::size_t port_id)
{
    if (reqPort.sendTimingReq(pkt)) {
        pktId2Port[pkt->id] = port_id;
        return true;
    }

    if (port_id ==0) {
        DPRINTF(TrafficMuxDebug, "schedule retry %ld\n", 0);
        schedule(event, curTick() + 250);
    }

    return false;
}

void
TrafficMux::processRetry()
{
    DPRINTF(TrafficMuxDebug, "process retry\n");
    rspPorts[0].sendRetryReq();
}

// RequestPort

bool
TrafficMux::recvTimingResp(PacketPtr pkt)
{
    PacketId id = pkt->id;
    auto itr = pktId2Port.find(id);

    panic_if(itr == pktId2Port.end(), "response to an unknown pkt");

    return rspPorts[itr->second].sendTimingResp(pkt);
}

void
TrafficMux::recvRangeChange()
{
    for (auto p : rspPorts)
        p.sendRangeChange();
}

Port &
TrafficMux::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "req_port") {
      return reqPort;
    } else if (if_name == "rsp_ports" && idx < rspPorts.size()) {
        // We should have already created all of the ports in the constructor
        return rspPorts[idx];
    }
    return ClockedObject::getPort(if_name, idx);
}

void
TrafficMux::switchOn()
{
    isActivated = true;
}

void
TrafficMux::switchOff()
{
    isActivated = false;
}

}; // namespace gem5
