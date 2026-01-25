/*
 * Copyright (c) 2024 The Regents of the University of California
 * All rights reserved
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __PERF_RUBY_DATA_MOVEMENT_TRACKER_HH__
#define __PERF_RUBY_DATA_MOVEMENT_TRACKER_HH__

#include <unordered_map>

#include "mem/cache/simple_cache_probe_arg.hh"
#include "mem/ruby/common/MachineID.hh"
#include "mem/ruby/slicc_interface/AbstractController.hh"
#include "mem/ruby/slicc_interface/RubyRequest.hh"
#include "sim/probe/probe.hh"

namespace gem5
{

namespace ruby
{

/**
 * This is a probe-triggered proxy for data movement tracker SimObject.
 * This wrapper enables the cache coherence protocol to probe the origin
 * and the destination of every memory request in the watch ranges.
 */
class RubyDataMovementTrackerProxy : public SimpleCacheAccessor, public Named
{
  public:
    RubyDataMovementTrackerProxy(AbstractController* cacheController);

    // Notification for cache fill events (writebacks)
    void notifyWriteback(
        const RequestPtr& req, const MachineID& requestor_id,
        const MachineID data_sender_id, const bool data_send_id_valid,
        const Tick latency, const DataBlock& data_blk,
        const unsigned cache_state
    );
    // Notification for cache hit events
    void notifyHit(
        const RequestPtr& req, const MachineID machine_id, const Addr addr,
        const unsigned cache_state, const DataBlock& data_blk
    );
    // Notification for hit from memory events
    void notifyHitFromMemory(
        const RequestPtr& req, const MachineID machine_id, const Addr addr
    );
    // Notification for cache miss events
    void notifyMiss(
        const RequestPtr& req, const MachineID requestor_id, const Addr addr,
        const unsigned cache_state
    );
    void regProbePoints();

  private:
    AbstractController* cacheController;

    ProbePointArg<SimpleCacheAccessProbeArg> *ppWriteback;
    ProbePointArg<SimpleCacheAccessProbeArg> *ppHit;
    ProbePointArg<SimpleCacheAccessProbeArg> *ppHitFromMemory;
    ProbePointArg<SimpleCacheAccessProbeArg> *ppMiss;

    Addr makeLineAddress(Addr addr) const;
    Addr getOffset(Addr addr) const;
};

} // namespace ruby
} // namespace gem5

 #endif // __PERF_RUBY_DATA_MOVEMENT_TRACKER_HH__
