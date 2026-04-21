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

#include "perf/RubyCacheBlockTracker/RubyCacheBlockTracker.hh"

#include <sstream>
#include <string>

#include "base/logging.hh"
#include "base/statistics.hh"
#include "base/stats/group.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "debug/RubyCacheBlockTrackerDebug.hh"
#include "debug/RubyCacheBlockTrackerObserverDebug.hh"
#include "mem/cache/simple_cache_probe_arg.hh"
#include "mem/request.hh"
#include "params/RubyCacheBlockTracker.hh"
#include "sim/probe/probe.hh"
#include "sim/sim_object.hh"
#include "sim/system.hh"

namespace gem5
{

namespace ruby
{

constexpr uint64_t log2(uint64_t n) {
    return (n < 2) ? 0 : 1 + log2(n / 2);
}

constexpr Addr getBlockAlignedAddr(const Addr addr, const Addr offset_bits) {
    return (addr >> offset_bits) << offset_bits;
}

constexpr bool isBlockAligned(const Addr addr, const Addr offset_bits) {
    return getBlockAlignedAddr(addr, offset_bits) == addr;
}

RubyCacheBlockTracker::RubyCacheBlockTracker(const Params &p)
  : ProbeListenerObject(p),
    system(p.system),
    cache_line_offset(log2(p.system->cacheLineSize())),
    usefulnessAttributionStats(this, p.system->cacheLineSize()),
    trackerStats(this)
{
    inform("Cache line offset %d\n", cache_line_offset);
}

RubyCacheBlockTracker::~RubyCacheBlockTracker()
{
}

void
RubyCacheBlockTracker::registerDemandRequestor(SimObject *obj)
{
    RequestorID id = system->lookupRequestorId(obj->name());
    panic_if(
        id == Request::invldRequestorId,
        "Object %s is not registered as a requestor in the system.\n"
        "%s\n",
        obj->name(),
        getAllRequestorIDs().c_str()
    );
    usefulnessAttributionStats.registerCpuRequestor(id, obj->name());
    inform(
        "Added demand requestor with id %d and name %s\n", id,
        obj->name()
    );
}

void
RubyCacheBlockTracker::registerDemandRequestorWithSubrequestor(
    SimObject *obj, const std::string &subrequestor_name
)
{
    const std::string full_requestor_name =
        csprintf("%s.%s", obj->name(), subrequestor_name.c_str());
    RequestorID id = system->lookupRequestorId(full_requestor_name);
    panic_if(
        id == Request::invldRequestorId,
        "Object %s.%s is not registered as a requestor in the system.\n"
        "%s\n",
        obj->name(),
        subrequestor_name.c_str(),
        getAllRequestorIDs().c_str()
    );
    usefulnessAttributionStats.registerCpuRequestor(
        id, full_requestor_name.c_str()
    );
    inform(
        "Added demand requestor with id %d and name %s\n", id,
        full_requestor_name.c_str()
    );
}

void
RubyCacheBlockTracker::registerPrefetcherRequestor(SimObject *obj)
{
    RequestorID id = system->lookupRequestorId(obj->name());
    panic_if(
        id == Request::invldRequestorId,
        "Object %s is not registered as a requestor in the system.\n"
        "%s\n",
        obj->name(),
        getAllRequestorIDs().c_str()
    );
    usefulnessAttributionStats.registerPrefetcherRequestor(id, obj->name());
    inform(
        "Added prefetcher requestor with id %d and name %s\n", id,
        obj->name()
    );
}

void
RubyCacheBlockTracker::registerPrefetcherRequestorWithSubrequestor(
    SimObject *obj, const std::string &subrequestor_name
)
{
    const std::string full_requestor_name =
        csprintf("%s.%s", obj->name(), subrequestor_name.c_str());
    RequestorID id = system->lookupRequestorId(full_requestor_name);
    panic_if(
        id == Request::invldRequestorId,
        "Object %s.%s is not registered as a requestor in the system.\n"
        "%s\n",
        obj->name(),
        subrequestor_name.c_str(),
        getAllRequestorIDs().c_str()
    );
    usefulnessAttributionStats.registerPrefetcherRequestor(
        id, full_requestor_name.c_str()
    );
    inform(
        "Added prefetcher requestor with id %d and name %s\n", id,
        full_requestor_name.c_str()
    );
}

void
RubyCacheBlockTracker::registerEventProbe(
    SimObject *obj, const char *event_name
)
{
    ProbeManager *obj_pm = obj->getProbeManager();
    if (strcmp(event_name, "cpu outgoing data request") == 0) {
        listeners.push_back(new CpuRequestListener(this, obj_pm, event_name));
        obj_pm->addListener(event_name, *(listeners.back()));
    } else if (strcmp(event_name, "Directory entry allocation") == 0) {
        listeners.push_back(
            new DirEntryAllocationListener(this, obj_pm, event_name)
        );
        obj_pm->addListener(event_name, *(listeners.back()));
    } else if (strcmp(event_name, "Directory entry deallocation") == 0) {
        listeners.push_back(
            new DirEntryDeallocationListener(this, obj_pm, event_name)
        );
        obj_pm->addListener(event_name, *(listeners.back()));
    } else if (strcmp(event_name, "DataMovementWriteback") == 0) {
        listeners.push_back(
            new DataMovementListener(
                /* owner */ this,
                /* probe_manager */ obj_pm,
                /* event_name */ event_name,
                /* is_cache_fill */ true,
                /* is_cache_fill_from_evict */ false,
                /* is_cache_evict */ false
            )
        );
        obj_pm->addListener(event_name, *(listeners.back()));
    } else if (strcmp(event_name, "DataMovementWritebackFromEviction") == 0) {
        listeners.push_back(
            new DataMovementListener(
                /* owner */ this,
                /* probe_manager */ obj_pm,
                /* event_name */ event_name,
                /* is_cache_fill */ false,
                /* is_cache_fill_from_evict */ true,
                /* is_cache_evict */ false
            )
        );
        obj_pm->addListener(event_name, *(listeners.back()));
    } else if (strcmp(event_name, "DataMovementEviction") == 0) {
        listeners.push_back(
            new DataMovementListener(
                /* owner */ this,
                /* probe_manager */ obj_pm,
                /* event_name */ event_name,
                /* is_cache_fill */ false,
                /* is_cache_fill_from_evict */ false,
                /* is_cache_evict */ true
            )
        );
        obj_pm->addListener(event_name, *(listeners.back()));
    } else {
        panic(
            "Unsupported event name for RubyCacheBlockTracker: %s", event_name
        );
    }
}

void
RubyCacheBlockTracker::processCpuRequest(const RequestPtr &req)
{
    if (!req->hasPaddr()) {
        return;
    }
    if (req->requestorId() == Request::invldRequestorId) {
        return;
    }
    trackerStats.numTrackedDemandRequests++;
    const Addr paddr = req->getPaddr();
    const Addr block_aligned_paddr = getBlockAlignedAddr(
        paddr, cache_line_offset
    );
    RUBY_CACHE_BLOCK_TRACKER_OBSERVER_DEBUG(
        "Processing CPU request: addr=0x%lx (block=0x%lx), size=%d, "
        "requestor_id=%d\n",
        req->getPaddr(),
        block_aligned_paddr,
        req->getSize(),
        req->requestorId()
    );
    usefulnessAttributionStats.onBlockUsedByDemandRequest(
        block_aligned_paddr, req->requestorId()
    );
}

void
RubyCacheBlockTracker::processDirEntryAllocation(
    const Addr &addr, const RequestPtr &req
)
{
    trackerStats.numTrackedDirectoryEntryAllocations++;
    const Addr block_aligned_addr = getBlockAlignedAddr(
        addr, cache_line_offset
    );
    RUBY_CACHE_BLOCK_TRACKER_OBSERVER_DEBUG(
        "Processing directory entry allocation: addr=0x%lx (block=0x%lx), "
        "size=%d, requestor_id=%d\n",
        addr,
        block_aligned_addr,
        req->hasSize() ? req->getSize() : 0,
        req->requestorId()
    );
    usefulnessAttributionStats.onBlockBroughtIntoCache(
        block_aligned_addr, req->requestorId()
    );
}

void
RubyCacheBlockTracker::processDirEntryDeallocation(const Addr &addr)
{
    trackerStats.numTrackedDirectoryEntryDeallocations++;
    const Addr block_aligned_addr = getBlockAlignedAddr(
        addr, cache_line_offset
    );
    RUBY_CACHE_BLOCK_TRACKER_OBSERVER_DEBUG(
        "Processing directory entry deallocation: addr=0x%lx (block=0x%lx)\n",
        addr,
        block_aligned_addr
    );
}

void
RubyCacheBlockTracker::processCacheFill(const SimpleCacheAccessProbeArg &arg)
{
    trackerStats.numTrackedCacheFills++;
    const Addr paddr = arg.req->getPaddr();
    const Addr block_aligned_paddr = getBlockAlignedAddr(
        paddr, cache_line_offset
    );
    RUBY_CACHE_BLOCK_TRACKER_OBSERVER_DEBUG(
        "Processing cache fill: addr=0x%lx (block=0x%lx), size=%d, "
        "requestor_id=%d\n",
        paddr,
        block_aligned_paddr,
        arg.req->getSize(),
        arg.req->requestorId()
    );
}

void
RubyCacheBlockTracker::processCacheFillFromEviction(
    const SimpleCacheAccessProbeArg &arg
)
{
    trackerStats.numTrackedCacheFillFromEviction++;
    const Addr addr = arg.eviction_addr;
    const Addr block_aligned_addr = getBlockAlignedAddr(
        addr, cache_line_offset
    );
    RUBY_CACHE_BLOCK_TRACKER_OBSERVER_DEBUG(
        "Processing cache fill from eviction: addr=0x%lx (block=0x%lx)\n",
        addr,
        block_aligned_addr
    );
}

void
RubyCacheBlockTracker::processCacheEviction(
    const SimpleCacheAccessProbeArg &arg
)
{
    trackerStats.numTrackedCacheEvictions++;
    const Addr addr = arg.eviction_addr;
    const Addr block_aligned_addr = getBlockAlignedAddr(
        addr, cache_line_offset
    );
    RUBY_CACHE_BLOCK_TRACKER_OBSERVER_DEBUG(
        "Processing cache eviction: addr=0x%lx (block=0x%lx)\n",
        addr,
        block_aligned_addr
    );
    usefulnessAttributionStats.onBlockEvictedFromCache(block_aligned_addr);
}

std::string
RubyCacheBlockTracker::getAllRequestorIDs() const
{
    std::stringstream strm;
    const uint64_t num_requestors = system->maxRequestors();
    for (RequestorID id = 0; id < num_requestors; ++id) {
        std::string requestor_name = system->getRequestorName(id);
        strm << "Requestor id " << id << " is registered with name "
             << requestor_name << "\n";
    }
    return strm.str();
}

RubyCacheBlockTracker::UsefulnessAttributionStats::UsefulnessAttributionStats(
    statistics::Group *parent,
    const Addr _cache_line_size
) : statistics::Group(parent),
    ADD_STAT(
        numUsefulBlocksBroughtIntoCacheByCpus, statistics::units::Count::get(),
        "Number of useful blocks that are brought into cache by CPUs"
    ),
    ADD_STAT(
        numUselessBlocksBroughtIntoCacheByCpus,
        statistics::units::Count::get(),
        "Number of useless blocks that are brought into cache by CPUs"
    ),
    ADD_STAT(
        numUsefulBlocksBroughtIntoCacheByPrefetchers,
        statistics::units::Count::get(),
        "Number of useful blocks that are brought into cache by prefetchers"
    ),
    ADD_STAT(
        numUselessBlocksBroughtIntoCacheByPrefetchers,
        statistics::units::Count::get(),
        "Number of useless blocks that are brought into cache by prefetchers"
    ),
    ADD_STAT(
        numUntrackedEvictions,
        statistics::units::Count::get(),
        "Number of eviction events that we don't know who brought the block in"
    ),
    cache_line_offset(log2(_cache_line_size))
{
    inform("Cache line offset %d\n", cache_line_offset);
}

void
RubyCacheBlockTracker::UsefulnessAttributionStats::regStats()
{
    statistics::Group::regStats();

    for (
        auto it = prefetcherRequestorIDs.begin();
        it != prefetcherRequestorIDs.end();
        ++it
    ) {
        RequestorID id = it->first;
        std::string name = it->second;
        numUsefulBlocksBroughtIntoCachePerPrefetcher[id] =
            new statistics::Scalar(
                this,
                csprintf(
                    "useful_blocks_brought_into_cache_by_prefetcher_%d", id
                ).c_str(),
                statistics::units::Count::get(),
                csprintf(
                    "Number of useful blocks that are brought into cache by "
                    "prefetcher %s", name.c_str()
                ).c_str()
            );
        numUselessBlocksBroughtIntoCachePerPrefetcher[id] =
            new statistics::Scalar(
                this,
                csprintf(
                    "useless_blocks_brought_into_cache_by_prefetcher_%d", id
                ).c_str(),
                statistics::units::Count::get(),
                csprintf(
                    "Number of useless blocks that are brought into cache by "
                    "prefetcher %s", name.c_str()
                ).c_str()
            );
    }
}

void
RubyCacheBlockTracker::UsefulnessAttributionStats::preDumpStats()
{
    statistics::Group::preDumpStats();
    // We can't emulate evicting cache blocks here as we might continue to run
    // the simulation after dumping stats, so we just update the stats
    // internally for the blocks that are still in the cache system.
    for (auto it = blockToFirstRequestorMap.begin();
         it != blockToFirstRequestorMap.end(); ++it) {
        Addr block_addr = it->first;
        updateUsefulnessStatsForBlock(block_addr);
    }

}

void
RubyCacheBlockTracker::UsefulnessAttributionStats::registerCpuRequestor(
    RequestorID id, const std::string &name
)
{
    cpuRequestorIDs.insert(std::make_pair(id, name));
}

void
RubyCacheBlockTracker::UsefulnessAttributionStats::registerPrefetcherRequestor(
    RequestorID id, const std::string &name
)
{
    prefetcherRequestorIDs.insert(std::make_pair(id, name));
}

void
RubyCacheBlockTracker::UsefulnessAttributionStats::onBlockBroughtIntoCache(
    const Addr block_addr, const RequestorID requestor_id
)
{
    assert(isBlockAligned(block_addr, cache_line_offset));
    // We need to check if the block is already in the cache system.
    auto it = blockToFirstRequestorMap.find(block_addr);
    if (it == blockToFirstRequestorMap.end()) {
        // The block was not in the cache system. We record the requestor ID as
        // the first requestor that brings the block into the cache system.
        blockToFirstRequestorMap[block_addr] = requestor_id;
        blockUsageCountMap[block_addr] = 0;
    } else {
        // The block is already in the cache system. Do nothing.
    }
}

void
RubyCacheBlockTracker::UsefulnessAttributionStats::onBlockEvictedFromCache(
    const Addr block_addr
)
{
    assert(isBlockAligned(block_addr, cache_line_offset));
    // First, we update the usefulness attribution stats.
    updateUsefulnessStatsForBlock(block_addr);
    // Finally, we remove the block from the tracking maps.
    blockToFirstRequestorMap.erase(block_addr);
    blockUsageCountMap.erase(block_addr);
}

void
RubyCacheBlockTracker::UsefulnessAttributionStats::onBlockUsedByDemandRequest(
    const Addr block_addr, const RequestorID requestor_id
)
{
    assert(isBlockAligned(block_addr, cache_line_offset));
    // If the block is already in the cache system, we increment the usage
    // count for the block.
    auto it = blockUsageCountMap.find(block_addr);
    if (it != blockUsageCountMap.end()) {
        it->second++;
    } else {
        // The block is not in the cache system. This is a demand request, so
        // we will credit the CPU as the requestor that brought the block into
        // the cache system.
        // Note: this is not perfect as the prefetcher might already make a
        // request for the block and bring it into the cache system before the
        // demand request comes in. Though, this window is tight as the
        // directory entry allocation happens right after Initiate_Request.
        blockToFirstRequestorMap[block_addr] = requestor_id;
        blockUsageCountMap[block_addr] = 1;
    }
}

void
RubyCacheBlockTracker::
UsefulnessAttributionStats::updateUsefulnessStatsForBlock(
    const Addr block_addr
)
{
    assert(isBlockAligned(block_addr, cache_line_offset));
    auto it = blockToFirstRequestorMap.find(block_addr);
    if (it == blockToFirstRequestorMap.end()) {
        // The block is not in the cache system. This should not happen as we
        // should only receive eviction events for blocks that are in the cache
        // system. We print a warning and return.
        warn_once(
            "Received eviction event for block address 0x%lx that is not in "
            "the cache system. This should not happen.\n", block_addr
        );
        numUntrackedEvictions++;
        assert(
            blockUsageCountMap.find(block_addr) == blockUsageCountMap.end()
        );
        return;
    }
    RequestorID first_requestor_id = it->second;
    uint64_t usage_count = blockUsageCountMap[block_addr];
    if (usage_count > 0) {
        requestorUsefulBlocksMap[first_requestor_id] += 1;
    } else {
        requestorUselessBlocksMap[first_requestor_id] += 1;
    }
    if (cpuRequestorIDs.find(first_requestor_id) != cpuRequestorIDs.end()) {
        if (usage_count > 0) {
            numUsefulBlocksBroughtIntoCacheByCpus++;
        } else {
            // Should not happen by definition :D
            numUselessBlocksBroughtIntoCacheByCpus++;
        }
    }
    if (prefetcherRequestorIDs.find(first_requestor_id) !=
        prefetcherRequestorIDs.end()) {
        if (usage_count > 0) {
            numUsefulBlocksBroughtIntoCacheByPrefetchers++;
            (*numUsefulBlocksBroughtIntoCachePerPrefetcher[
                first_requestor_id
            ])++;
        } else {
            numUselessBlocksBroughtIntoCacheByPrefetchers++;
            (*numUselessBlocksBroughtIntoCachePerPrefetcher[
                first_requestor_id
            ])++;
        }
    }
}

RubyCacheBlockTracker::TrackerStats::TrackerStats(statistics::Group *parent)
  : statistics::Group(parent),
    ADD_STAT(
        numTrackedDemandRequests, statistics::units::Count::get(),
        "Number of demand requests tracked"
    ),
    ADD_STAT(
        numTrackedDirectoryEntryAllocations, statistics::units::Count::get(),
        "Number of directory entry allocations tracked"
    ),
    ADD_STAT(
        numTrackedDirectoryEntryDeallocations, statistics::units::Count::get(),
        "Number of directory entry deallocations tracked"
    ),
    ADD_STAT(
        numTrackedCacheFills, statistics::units::Count::get(),
        "Number of cache fills tracked"
    ),
    ADD_STAT(
        numTrackedCacheFillFromEviction, statistics::units::Count::get(),
        "Number of cache fills from eviction tracked"
    ),
    ADD_STAT(
        numTrackedCacheEvictions, statistics::units::Count::get(),
        "Number of cache evictions tracked"
    )
{
}

RubyCacheBlockTracker::
CpuRequestListener::CpuRequestListener(
    RubyCacheBlockTracker *_owner, ProbeManager *_probe_manager,
    const char *_name
) : ProbeListenerArgBase<RequestPtr>(_probe_manager, _name), owner(_owner)
{
}

void
RubyCacheBlockTracker::CpuRequestListener::notify(const RequestPtr &req)
{
    owner->processCpuRequest(req);
}

RubyCacheBlockTracker::DirEntryAllocationListener::DirEntryAllocationListener(
    RubyCacheBlockTracker *_owner, ProbeManager *_probe_manager,
    const char *_name
) : ProbeListenerArgBase<std::pair<Addr, RequestPtr>>(_probe_manager, _name),
    owner(_owner)
{
}

void
RubyCacheBlockTracker::DirEntryAllocationListener::notify(
    const std::pair<Addr, RequestPtr> &arg
)
{
    owner->processDirEntryAllocation(arg.first, arg.second);
}

RubyCacheBlockTracker::
DirEntryDeallocationListener::DirEntryDeallocationListener(
    RubyCacheBlockTracker *_owner, ProbeManager *_probe_manager,
    const char *_name
) : ProbeListenerArgBase<Addr>(_probe_manager, _name), owner(_owner)
{
}

void
RubyCacheBlockTracker::DirEntryDeallocationListener::notify(const Addr &arg)
{
    owner->processDirEntryDeallocation(arg);
}

RubyCacheBlockTracker::DataMovementListener::DataMovementListener(
    RubyCacheBlockTracker *_owner, ProbeManager *_probe_manager,
    const char *_name, const bool _is_cache_fill,
    const bool _is_cache_fill_from_evict, const bool _is_cache_evict
) : ProbeListenerArgBase<SimpleCacheAccessProbeArg>(_probe_manager, _name),
    owner(_owner), is_cache_fill(_is_cache_fill),
    is_cache_fill_from_evict(_is_cache_fill_from_evict),
    is_cache_evict(_is_cache_evict)
{
    const bool only_cache_fill =
        is_cache_fill && !is_cache_fill_from_evict && !is_cache_evict;
    const bool only_cache_fill_from_evict =
        !is_cache_fill && is_cache_fill_from_evict && !is_cache_evict;
    const bool only_cache_evict =
        !is_cache_fill && !is_cache_fill_from_evict && is_cache_evict;
    panic_if(
        !(only_cache_fill || only_cache_evict || only_cache_fill_from_evict),
        "DataMovementListener must be exactly one of the following: "
        "cache fill, cache eviction, or receiving eviction"
    );
}

void
RubyCacheBlockTracker::DataMovementListener::notify(
    const SimpleCacheAccessProbeArg &arg
)
{
    if (is_cache_fill) {
        owner->processCacheFill(arg);
    } else if (is_cache_fill_from_evict) {
        owner->processCacheFillFromEviction(arg);
    } else if (is_cache_evict) {
        owner->processCacheEviction(arg);
    } else {
        panic("DataMovementListener must be either cache fill or cache evict");
    }
}

}  // namespace ruby

}  // namespace gem5
