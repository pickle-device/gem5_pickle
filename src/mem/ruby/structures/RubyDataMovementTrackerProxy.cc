/*
 * Copyright (c) 2024 The Regents of the University of California
 * All rights reserved
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "mem/ruby/structures/RubyDataMovementTrackerProxy.hh"

#include <iomanip>
#include <iostream>

#include "mem/ruby/system/RubySystem.hh"

namespace gem5
{

namespace ruby
{

RubyDataMovementTrackerProxy::RubyDataMovementTrackerProxy(
    AbstractController* _cacheController
) : Named(_cacheController->name()),
    cacheController(_cacheController),
    ppWriteback(nullptr),
    ppHit(nullptr),
    ppAccess(nullptr)
{
    fatal_if(
        !cacheController,
        "A RubyDataMovementTrackerProxy is not attached to a CacheController"
    );
}

void
RubyDataMovementTrackerProxy::notifyWriteback(
    const RequestPtr& req, const MachineID& requestor_id,
    const MachineID data_sender_id, const bool data_sender_id_valid,
    const Tick latency, const DataBlock& data_blk, const unsigned cache_state
)
{
    assert(req);
    RequestPtr req_copy(new Request(*req));
    ppWriteback->notify(SimpleCacheAccessProbeArg(
        req_copy, *this, data_sender_id, data_sender_id_valid, latency,
        cache_state)
    );
}

void
RubyDataMovementTrackerProxy::notifyHit(
    const RequestPtr& req, const MachineID machine_id, const Addr addr,
    const unsigned cache_state
)
{
    assert(req);
    RequestPtr req_copy(new Request(*req));
    ppHit->notify(SimpleCacheAccessProbeArg(
        req_copy, *this, machine_id, true, 0, cache_state
    ));
}

void
RubyDataMovementTrackerProxy::notifyAccess(
    const RequestPtr& req, const MachineID machine_id, const Addr addr,
    const unsigned cache_state
)
{
    assert(req);
    RequestPtr req_copy(new Request(*req));
    ppAccess->notify(SimpleCacheAccessProbeArg(
        req_copy, *this, machine_id, true, 0, cache_state
    ));
}

void
RubyDataMovementTrackerProxy::regProbePoints()
{
    ppWriteback = new ProbePointArg<SimpleCacheAccessProbeArg>(
        cacheController->getProbeManager(), "DataMovementWriteback"
    );
    ppHit = new ProbePointArg<SimpleCacheAccessProbeArg>(
        cacheController->getProbeManager(), "DataMovementHit"
    );
    ppAccess = new ProbePointArg<SimpleCacheAccessProbeArg>(
        cacheController->getProbeManager(), "DataMovementAccess"
    );
}

Addr
RubyDataMovementTrackerProxy::makeLineAddress(Addr addr) const
{
    return ruby::makeLineAddress(
        addr, cacheController->m_ruby_system->getBlockSizeBits()
    );
}

Addr
RubyDataMovementTrackerProxy::getOffset(Addr addr) const
{
    return ruby::getOffset(
        addr, cacheController->m_ruby_system->getBlockSizeBits()
    );
}

} // namespace ruby
} // namespace gem5
