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

#include "pickle/gadgets/traffic_snooper.hh"

#include "debug/TrafficSnooperDebug.hh"

namespace gem5
{

TrafficSnooper::TrafficSnooper(const TrafficSnooperParams &params)
  : ClockedObject(params),
    isActivated(false), watch_ranges(params.watch_ranges)
{
    in_port = new TrafficSnooperResponsePort(".in_port", *this, 0);
    out_port = new TrafficSnooperRequestPort(".out_port", *this, 0);
    //snoop_port = new TrafficSnooperPassiveRequestPort(".snoop_port",*this,0);
    snoop_port = new TrafficSnooperRequestPort(".snoop_port", *this, 0);
}

TrafficSnooper::~TrafficSnooper()
{
    delete snoop_port;
    delete out_port;
    delete in_port;
}

void
TrafficSnooper::startup()
{
}

bool
TrafficSnooper::recvTimingReq(PacketPtr pkt)
{
    if (isActivated
        && pkt->req->isUncacheable()
        && inRange(pkt->req->getPaddr())) {
        DPRINTF(
            TrafficSnooperDebug,
            "Snooping timing packet 0x%llx\n", pkt->req->getPaddr());
        return snoop_port->sendTimingReq(pkt);
    } else {
        return out_port->sendTimingReq(pkt);
    }
}

bool
TrafficSnooper::recvTimingSnoopResp(PacketPtr pkt)
{
    return out_port->sendTimingSnoopResp(pkt);
}

Tick
TrafficSnooper::recvAtomic(PacketPtr pkt)
{
    if (isActivated
        && pkt->req->isUncacheable()
        && inRange(pkt->req->getPaddr())) {
        DPRINTF(
            TrafficSnooperDebug,
            "Snooping atomic packet 0x%llx\n", pkt->req->getPaddr()
        );
        return snoop_port->sendAtomic(pkt);
    } else {
        return out_port->sendAtomic(pkt);
    }
}

Tick
TrafficSnooper::recvAtomicBackdoor(PacketPtr pkt, MemBackdoorPtr &backdoor)
{
    // ??? Fix this logic
    if (isActivated && pkt->hasData() && pkt->cmd.isWrite())
    {
        PacketPtr new_pkt = duplicatePkt(pkt);
        snoop_port->sendAtomicBackdoor(new_pkt, backdoor);
    }
    return out_port->sendAtomicBackdoor(pkt, backdoor);
}

void
TrafficSnooper::recvFunctional(PacketPtr pkt)
{
    if (isActivated && pkt->hasData() && pkt->cmd.isWrite())
    {
        PacketPtr new_pkt = duplicatePkt(pkt);
        snoop_port->sendFunctional(new_pkt);
        DPRINTF(
            TrafficSnooperDebug,
            "Snooping functional packet 0x%llx\n",
            pkt->req->getPaddr()
        );
    }
    out_port->sendFunctional(pkt);
}

void
TrafficSnooper::recvMemBackdoorReq(
  const MemBackdoorReq &req, MemBackdoorPtr &backdoor)
{
    out_port->sendMemBackdoorReq(req, backdoor);
}

AddrRangeList
TrafficSnooper::getAddrRanges() const
{
    return out_port->getAddrRanges();
}

void
TrafficSnooper::recvRespRetry()
{
    out_port->sendRetryResp();
}

bool
TrafficSnooper::recvTimingResp(PacketPtr pkt)
{
    return in_port->sendTimingResp(pkt);
}

void
TrafficSnooper::recvTimingSnoopReq(PacketPtr pkt)
{
    in_port->sendTimingSnoopReq(pkt);
}

Tick
TrafficSnooper::recvAtomicSnoop(PacketPtr pkt)
{
    return in_port->sendAtomicSnoop(pkt);
}

void
TrafficSnooper::recvFunctionalSnoop(PacketPtr pkt)
{
    in_port->sendFunctionalSnoop(pkt);
}

void
TrafficSnooper::recvRangeChange()
{
    in_port->sendRangeChange();
}

void
TrafficSnooper::recvReqRetry()
{
    in_port->sendRetryReq();
}

Port &
TrafficSnooper::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "in_port")
      return *in_port;
    else if (if_name == "out_port")
      return *out_port;
    else if (if_name == "snoop_port")
      return *snoop_port;
    return ClockedObject::getPort(if_name, idx);
}

PacketPtr
TrafficSnooper::duplicatePkt(PacketPtr pkt) const
{
    // we don't know the lifetime of the original packet so it's safer to
    // duplicate the packet and then the new one to the snooping port
    PacketPtr new_pkt = new Packet(pkt, true, true);
    new_pkt->setData(pkt->getPtr<uint8_t>());
    return new_pkt;
    //return pkt;
}

void
TrafficSnooper::switchOn()
{
    isActivated = true;
    DPRINTF(TrafficSnooperDebug, "Switch On\n");
}

void
TrafficSnooper::switchOff()
{
    isActivated = false;
}

void
TrafficSnooper::addWatchRange(AddrRange range)
{
    watch_ranges.push_back(range);
}
bool
TrafficSnooper::inRange(Addr addr)
{
    for (auto r : watch_ranges) {
        if (r.contains(addr))
            return true;
    }

    return false;
}

}; // namespace gem5
