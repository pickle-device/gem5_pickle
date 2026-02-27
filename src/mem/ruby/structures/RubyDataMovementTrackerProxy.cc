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
    ppMiss(nullptr)
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
    const uint64_t data_size = req->getSize();
    const uint8_t* data = data_blk.getData(
        /*Offset*/ getOffset(req->getPaddr()),
        /*Length*/ data_size
    );
    std::vector<uint8_t> cache_fill_data(data, data + data_size);
    ppWriteback->notify(SimpleCacheAccessProbeArg(
        /*request*/ req_copy,
        /*eviction_addr*/ 0,
        /*cache_accessor*/ *this,
        /*machineID*/ data_sender_id, // where did we get the data from
        /*machineIDValid*/ data_sender_id_valid,
        /*latency*/ latency,
        /*cache_state*/ cache_state,
        /*cache_fill_data*/ std::move(cache_fill_data)
    ));
}

void
RubyDataMovementTrackerProxy::notifyHit(
    const RequestPtr& req, const MachineID machine_id, const Addr addr,
    const unsigned cache_state, const DataBlock& data_blk
)
{
    assert(req);
    RequestPtr req_copy(new Request(*req));
    const uint64_t data_size = req->getSize();
    const uint8_t* data = data_blk.getData(
        /*Offset*/ getOffset(req->getPaddr()),
        /*Length*/ data_size
    );
    std::vector<uint8_t> cache_fill_data(data, data + data_size);
    ppHit->notify(SimpleCacheAccessProbeArg(
        /*request*/ req_copy,
        /*eviction_addr*/ 0,
        /*cache_accessor*/ *this,
        /*machineID*/ machine_id,
        /*machineIDValid*/ true,
        /*latency*/ 0,
        /*cache_state*/ cache_state,
        /*cache_fill_data*/ std::move(cache_fill_data)
    ));
}

void
RubyDataMovementTrackerProxy::notifyHitFromMemory(
    const RequestPtr& req, const MachineID machine_id, const Addr addr
)
{
    assert(req);
    RequestPtr req_copy(new Request(*req));
    ppHit->notify(SimpleCacheAccessProbeArg(
        /*request*/ req_copy,
        /*eviction_addr*/ 0,
        /*cache_accessor*/ *this,
        /*machineID*/ machine_id,
        /*machineIDValid*/ true,
        /*latency*/ 0,
        /*cache_state*/ 0,
        /*cache_fill_data*/ {}
    ));
}

void
RubyDataMovementTrackerProxy::notifyMiss(
    const RequestPtr& req, const MachineID machine_id, const Addr addr,
    const unsigned cache_state
)
{
    assert(req);
    RequestPtr req_copy(new Request(*req));
    ppMiss->notify(SimpleCacheAccessProbeArg(
        /*request*/ req_copy,
        /*eviction_addr*/ 0,
        /*cache_accessor*/ *this,
        /*machineID*/ machine_id,
        /*machineIDValid*/ true,
        /*latency*/ 0,
        /*cache_state*/ cache_state,
        /*cache_fill_data*/ {}
    ));
}

void
RubyDataMovementTrackerProxy::notifyEviction(
    const MachineID machine_id, const Addr addr
)
{
    ppEviction->notify(SimpleCacheAccessProbeArg(
        /*request*/ nullptr,
        /*eviction_addr*/ addr,
        /*cache_accessor*/ *this,
        /*machineID*/ machine_id,
        /*machineIDValid*/ true,
        /*latency*/ 0,
        /*cache_state*/ 0,
        /*cache_fill_data*/ {}
    ));
}

void
RubyDataMovementTrackerProxy::notifyWritebackFromEviction(
    const MachineID machine_id, const Addr addr
)
{
    ppWritebackFromEviction->notify(SimpleCacheAccessProbeArg(
        /*request*/ nullptr,
        /*eviction_addr*/ addr,
        /*cache_accessor*/ *this,
        /*machineID*/ machine_id,
        /*machineIDValid*/ true,
        /*latency*/ 0,
        /*cache_state*/ 0,
        /*cache_fill_data*/ {}
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
    ppHitFromMemory = new ProbePointArg<SimpleCacheAccessProbeArg>(
        cacheController->getProbeManager(), "DataMovementHitFromMemory"
    );
    ppMiss = new ProbePointArg<SimpleCacheAccessProbeArg>(
        cacheController->getProbeManager(), "DataMovementMiss"
    );
    ppEviction = new ProbePointArg<SimpleCacheAccessProbeArg>(
        cacheController->getProbeManager(), "DataMovementEviction"
    );
    ppWritebackFromEviction = new ProbePointArg<SimpleCacheAccessProbeArg>(
        cacheController->getProbeManager(), "DataMovementWritebackFromEviction"
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
