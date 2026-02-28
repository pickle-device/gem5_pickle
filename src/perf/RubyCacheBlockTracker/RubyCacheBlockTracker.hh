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

#include <set>
#include <string>
#include <utility>

#include "base/statistics.hh"
#include "base/stats/group.hh"
#include "base/types.hh"
#include "debug/RubyCacheBlockTrackerDebug.hh"
#include "debug/RubyCacheBlockTrackerObserverDebug.hh"
#include "mem/cache/simple_cache_probe_arg.hh"
#include "mem/request.hh"
#include "params/RubyCacheBlockTracker.hh"
#include "sim/probe/probe.hh"
#include "sim/sim_object.hh"
#include "sim/system.hh"

#define RUBY_CACHE_BLOCK_TRACKER_DEBUG(...) \
    DPRINTF(RubyCacheBlockTrackerDebug, __VA_ARGS__)

// This observer debug is for debugging the observable events probed by the
// tracker.
#define RUBY_CACHE_BLOCK_TRACKER_OBSERVER_DEBUG(...) \
    DPRINTF(RubyCacheBlockTrackerObserverDebug, __VA_ARGS__)

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
    // void init() override; // already overriden in Python
    // void regProbePoints() override; // already overridden in Python
    void registerEventProbe(SimObject *obj, const char *event_name);
    void registerDemandRequestor(SimObject *obj);
    void registerPrefetcherRequestor(SimObject *obj);

    void processCpuRequest(const RequestPtr &req);
    void processDirEntryAllocation(const Addr &addr, const RequestPtr &req);
    void processDirEntryDeallocation(const Addr &addr);
    void processCacheFill(const SimpleCacheAccessProbeArg &arg);
    void processCacheFillFromEviction(const SimpleCacheAccessProbeArg &arg);
    void processCacheEviction(const SimpleCacheAccessProbeArg &arg);

  private:
    std::string getAllRequestorIDs() const;

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
    class UsefulnessAttributionStats : public statistics::Group
    {
      public:
        UsefulnessAttributionStats(statistics::Group *parent);
        void regStats() override;
        void preDumpStats() override;

        void registerCpuRequestor(RequestorID id, const std::string &name);
        void registerPrefetcherRequestor(
          RequestorID id, const std::string &name
        );

        void onBlockBroughtIntoCache(
          const Addr block_addr, const RequestorID requestor_id
        );
        void onBlockEvictedFromCache(const Addr block_addr);
        void onBlockUsedByDemandRequest(
          const Addr block_addr, const RequestorID requestor_id
        );
      private:
         void updateUsefulnessStatsForBlock(const Addr block_addr);

      public:
        /* OVERALL STATS */
        statistics::Scalar numUsefulBlocksBroughtIntoCacheByCpus;
        statistics::Scalar numUselessBlocksBroughtIntoCacheByCpus;
        statistics::Scalar numUsefulBlocksBroughtIntoCacheByPrefetchers;
        statistics::Scalar numUselessBlocksBroughtIntoCacheByPrefetchers;
        /* PER PREFETCHER STATS */
        std::map<RequestorID, statistics::Scalar *>
          numUsefulBlocksBroughtIntoCachePerPrefetcher;
        std::map<RequestorID, statistics::Scalar *>
          numUselessBlocksBroughtIntoCachePerPrefetcher;

      private:
        /* REQUESTORS */
        std::map<RequestorID, std::string> cpuRequestorIDs;
        std::map<RequestorID, std::string> prefetcherRequestorIDs;
        /* TRACKING BLOCKS */
        // Mapping the cache block address to the requestor ID of the first
        // request that brings the block into the cache system.
        std::unordered_map<Addr, RequestorID> blockToFirstRequestorMap;
        // The number of times that the block is used by demand requests after
        // it is brought into the cache system.
        std::unordered_map<Addr, uint64_t> blockUsageCountMap;
        /* TRACKING REQUESTORS */
        // Number of times a requestor brought a block into the cache system
        // and the block is used by at least one demand request after that.
        std::unordered_map<RequestorID, uint64_t> requestorUsefulBlocksMap;
        // Number of times a requestor brought a block into the cache system
        // and the block is not used by any demand request after that.
        std::unordered_map<RequestorID, uint64_t> requestorUselessBlocksMap;
    } usefulnessAttributionStats;

    struct TrackerStats : public statistics::Group
    {
      TrackerStats(statistics::Group *parent);

      statistics::Scalar numTrackedDemandRequests;
      statistics::Scalar numTrackedDirectoryEntryAllocations;
      statistics::Scalar numTrackedDirectoryEntryDeallocations;
      statistics::Scalar numTrackedCacheFills;
      statistics::Scalar numTrackedCacheFillFromEviction;
      statistics::Scalar numTrackedCacheEvictions;
    } trackerStats;

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
