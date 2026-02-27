/*
 * Copyright (c) 2026 The Regents of the University of California
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __PERF_RUBY_CACHE_BLOCK_TRACKER_HH__
#define __PERF_RUBY_CACHE_BLOCK_TRACKER_HH__

#include <utility>

#include "base/statistics.hh"
#include "base/stats/group.hh"
#include "base/types.hh"
#include "debug/RubyCacheBlockTrackerDebug.hh"
#include "mem/cache/simple_cache_probe_arg.hh"
#include "mem/request.hh"
#include "params/RubyCacheBlockTracker.hh"
#include "sim/probe/probe.hh"
#include "sim/sim_object.hh"
#include "sim/system.hh"

#define RUBY_CACHE_BLOCK_TRACKER_DEBUG(...) \
    DPRINTF(RubyCacheBlockTrackerDebug, __VA_ARGS__)

namespace gem5
{

namespace ruby
{

class RubyCacheBlockTracker : public ProbeListenerObject
{
  public:
    PARAMS(RubyCacheBlockTracker);
    RubyCacheBlockTracker(const Params &p);
    ~RubyCacheBlockTracker();
    void init() override;
    void addEventProbe(SimObject *obj, const char *event_name);

    void processCpuRequest(const RequestPtr &req);
    void processDirEntryAllocation(const Addr &addr, const RequestPtr &req);
    void processDirEntryDeallocation(const Addr &addr);
    void processCacheFill(const SimpleCacheAccessProbeArg &arg);
    void processCacheFillFromEviction(const SimpleCacheAccessProbeArg &arg);
    void processCacheEviction(const SimpleCacheAccessProbeArg &arg);

  private:
    System* system;

  public:
    // TODO: make sure that we track all blocks on chip. The LLC directory
    // only tracks the blocks that are L1 and L2, and not neccessarily all the
    // blocks in the LLC.
    // TODO: we might want to track the following,
    // 1) the distribution of the requestor IDs of the request that brings the
    //    blocks into the cache system for all demand requests. This can help
    //    us understand how many requests that the prefetcher is responsible
    //    for.
    // 2) same as above, but for private cache prefetcher, determine the
    //    distribution of the requestor IDs of the request that brings the
    //    blocks into the private cache.
    // 3) how many block that are brought into the cache system but are never
    //    consumed by any demand request. This can help us understand the
    //    usefulness of the prefetcher.
    struct UsefulnessAttributionStats : public statistics::Group
    {
      UsefulnessAttributionStats(statistics::Group *parent);
    } usefulnessAttributionStats;


  private:
    // Listeners
    class CpuRequestListener : public ProbeListenerArgBase<RequestPtr>
    {
      private:
        RubyCacheBlockTracker *owner;
      public:
        CpuRequestListener(
          RubyCacheBlockTracker *_owner, ProbeManager *_probe_manager,
          const char *_name
        );
        void notify(const RequestPtr &req) override;
    };  // class CpuRequestListener
    class DirEntryAllocationListener
      : public ProbeListenerArgBase<std::pair<Addr, RequestPtr>>
    {
      private:
        RubyCacheBlockTracker *owner;
      public:
        DirEntryAllocationListener(
          RubyCacheBlockTracker *_owner, ProbeManager *_probe_manager,
          const char *_name
        );
        void notify(const std::pair<Addr, RequestPtr> &arg) override;
    };  // class DirEntryAllocationListener
    class DirEntryDeallocationListener : public ProbeListenerArgBase<Addr>
    {
      private:
        RubyCacheBlockTracker *owner;
      public:
        DirEntryDeallocationListener(
          RubyCacheBlockTracker *_owner, ProbeManager *_probe_manager,
          const char *_name
        );
        void notify(const Addr &arg) override;
    };  // class DirEntryDeallocationListener
    class DataMovementListener
      : public ProbeListenerArgBase<SimpleCacheAccessProbeArg>
    {
      private:
        RubyCacheBlockTracker *owner;
        const bool is_cache_fill;
        const bool is_cache_fill_from_evict;
        const bool is_cache_evict;
      public:
        DataMovementListener(
          RubyCacheBlockTracker *_owner, ProbeManager *_probe_manager,
          const char *_name, const bool _is_cache_fill,
          const bool _is_cache_fill_from_evict, const bool _is_cache_evict
        );
        void notify(const SimpleCacheAccessProbeArg &arg) override;
    };  // class DataMovementListener
};  // class RubyCacheBlockTracker

}  // namespace ruby

}  // namespace gem5

#endif // __PERF_RUBY_CACHE_BLOCK_TRACKER_HH__
