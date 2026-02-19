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

#include "mem/cache/prefetch/differential_matching_prefetcher/differential_matching_prefetcher.hh"

#include <cassert>

#include "base/addr_range.hh"
#include "base/intmath.hh"
#include "base/logging.hh"
#include "base/random.hh"
#include "base/statistics.hh"
#include "base/trace.hh"
#include "debug/DifferentialMatchingPrefetcherCacheObserverDebug.hh"
#include "debug/DifferentialMatchingPrefetcherDebug.hh"
#include "enums/CacheLevel.hh"
#include "mem/cache/cache_probe_arg.hh"
#include "params/DifferentialMatchingPrefetcher.hh"
#include "sim/clock_domain.hh"
#include "sim/probe/probe.hh"

namespace gem5
{

namespace prefetch
{

namespace dmp
{

CpuRequestListener::CpuRequestListener(
    DifferentialMatchingPrefetcher *_owner, ProbeManager *_probe_manager,
    const char *name
) : ProbeListenerArgBase<RequestPtr>(_probe_manager, name),
    owner(_owner)
{}

void
CpuRequestListener::notify(const RequestPtr &req)
{
    owner->observeCpuOutgoingRequest(req);
}

CpuResponseListener::CpuResponseListener(
    DifferentialMatchingPrefetcher *_owner, ProbeManager *_probe_manager,
    const char *name
) : ProbeListenerArgBase<PacketPtr>(_probe_manager, name),
    owner(_owner)
{}

void
CpuResponseListener::notify(const PacketPtr &pkt)
{
    owner->observeCpuIncomingResponse(pkt);
}

CacheAccessListener::CacheAccessListener(
    DifferentialMatchingPrefetcher *_owner, ProbeManager *_probe_manager,
    const char *_name, bool _is_hit, bool _is_miss, bool _is_fill
) : ProbeListenerArgBase<SimpleCacheAccessProbeArg>(_probe_manager, _name),
    owner(_owner), is_hit(_is_hit), is_miss(_is_miss), is_fill(_is_fill)
{
    if (is_hit) {
        panic_if(is_miss, "Cache access cannot be both hit and miss");
        panic_if(is_fill, "Cache access cannot be both hit and fill");
    }
    if (is_miss) {
        panic_if(is_hit, "Cache access cannot be both hit and miss");
        panic_if(is_fill, "Cache access cannot be both miss and fill");
    }
    if (is_fill) {
        panic_if(is_hit, "Cache access cannot be both hit and fill");
        panic_if(is_miss, "Cache access cannot be both miss and fill");
    }
    if (!is_hit && !is_miss && !is_fill) {
        panic("Cache access must be either hit, miss or fill");
    }
}

void
CacheAccessListener::notify(const SimpleCacheAccessProbeArg &arg)
{
    if (is_hit) {
        owner->observeL1CacheHit(arg);
    } else if (is_miss) {
        owner->observeL1CacheMiss(arg);
    } else if (is_fill) {
        owner->observeL1CacheFill(arg);
    } else {
        panic("Invalid cache access type observed in CacheAccessListener");
    }
}

DifferentialMatchingPrefetcher::DifferentialMatchingPrefetcher(
    const DifferentialMatchingPrefetcherParams &p
) : ProbeListenerObject(p), system(p.system),
    cache_line_size(p.system->cacheLineSize()),
    clock_domain(p.clock_domain),
    memory_ranges(p.memory_ranges.begin(), p.memory_ranges.end()),
    dmp_prefetch_queue(p.dmp_prefetch_queue),
    stride_prefetch_queue(p.stride_prefetch_queue),
    l1_controller(p.l1_controller),
    l2_controller(p.l2_controller),
    enable_dmp_prefetching(p.enable_dmp_prefetching),
    index_queue_size(p.index_queue_size),
    indirection_candidate_scoreboard_num_entries(
        p.indirection_candidate_scoreboard_num_entries
    ),
    indirection_candidate_scoreboard_num_candidates_per_entry(
        p.indirection_candidate_scoreboard_num_candidates_per_entry
    ),
    sample_window_size(p.sample_window_size),
    ics_deprioritize_on_unsuccessful_matching_patch(
        p.ics_deprioritize_on_unsuccessful_matching_patch
    ),
    stride_tracker(
        /*capacity*/ p.stride_prefetcher_num_entries,
        /*_confidence_threshold*/ 0.5,
        /*_memory_ranges*/ memory_ranges,
        /*_cache_block_size*/ p.system->cacheLineSize(),
        /*_prefetch_distance*/ p.stride_prefetcher_distance,
        /*_prefetch_degree*/ p.stride_prefetcher_degree,
        /*_can_cross_page*/ p.stride_prefetcher_can_cross_page,
        /*_page_size_in_bytes*/ p.page_size,
        /*_stride_prefetch_pc_even_when_dmp_has_the_same_target_pc*/
        p.stride_prefetch_pc_even_when_dmp_has_the_same_target_pc,
        /*_prefetcher_interface*/ this
    ),
    index_queue(
        /*_max_size*/ p.index_queue_size,
        /*_replacement_policy*/ IndexQueueReplacementPolicy::LRU,
        /*_prefetcher_interface*/ this
    ),
    indirection_candidate_scoreboard(
        /*_max_num_entries*/ p.indirection_candidate_scoreboard_num_entries,
        /*_max_num_candidates*/
        p.indirection_candidate_scoreboard_num_candidates_per_entry,
        /*_sample_window_size*/
        p.sample_window_size,
        /*_deprioritize_previously_unsuccessful_match*/
        p.ics_deprioritize_on_unsuccessful_matching_patch,
        /*_prefetcher_interface*/
        this
    ),
    differential_matcher(
        /*_max_num_index_table_entries*/
        p.index_table_num_entries,
        /*_max_num_tracked_items_per_index_table_entry*/
        p.tracked_items_per_index_table_entry,
        /*_max_num_target_table_entries*/
        p.target_table_num_entries,
        /*_max_num_tracked_items_per_table_entry*/
        p.tracked_items_per_target_table_entry,
        /*_matching_shift_amounts*/
        p.matching_shift_amounts,
        /*_evict_stuck_entries*/
        p.evict_stuck_entries_patch,
        /*_stuck_entry_eviction_threshold_cycles*/
        p.stuck_entry_eviction_threshold_cycles,
        /*_clock_domain*/
        p.clock_domain,
        /*_prefetcher_interface*/
        this
    ),
    indirect_relation_table(
        /*_max_num_indirect_relation_entries*/
        p.indirect_relation_table_num_entries,
        /*_max_num_range_table_entries*/
        p.range_table_num_entries,
        /*_cache_block_size*/
        p.system->cacheLineSize(),
        /*_prefetcher_interface*/
        this
    ),
    stats(this)
{
    // We use 1KiB as a placeholder value for memory size because the actual
    // memory size is not known at the time of prefetcher construction in
    // Python. The actual memory size must be set later.
    panic_if(memory_ranges.empty(), "Memory ranges must be set");
    panic_if(l1_controller == nullptr,
            "L1 controller pointer passed to DMP prefetcher is null");
    panic_if(l2_controller == nullptr,
        "L2 controller pointer passed to DMP prefetcher is null");
    // this is not a very clean design as it creates a circular dependency
    // between the prefetcher and the prefetch queue, but it is simple and
    // works for our purpose.
    dmp_prefetch_queue->setOwner(this);
    dmp_prefetch_queue->setCacheController(l2_controller);
    dmp_prefetch_queue->setIndirectRelationTable(&indirect_relation_table);
    stride_prefetch_queue->setOwner(this);
    stride_prefetch_queue->setCacheController(l1_controller);
    stride_prefetch_queue->setIndirectRelationTable(nullptr);
    stride_tracker.setPrefetchQueue(stride_prefetch_queue);
}

void
DifferentialMatchingPrefetcher::promoteIndexPcFromIqToIcs()
{
    std::optional<std::vector<Addr>> pc_opt = index_queue.getHighestScorePcs();
    if (pc_opt.has_value()) {
        for (const Addr pc: pc_opt.value()) {
            // We add the PC to the ICS
            bool successfully_added = \
                indirection_candidate_scoreboard.addEntry(pc);
            if (successfully_added) {
                // We increase the tried count in the IQ
                index_queue.profileTriedCount(pc);
                DMP_PREFETCHER_DEBUG(
                    "Index PC %#x promoted from IQ to ICS\n", pc
                );
                break;
            }
        }
    }
}

void
DifferentialMatchingPrefetcher::addIndirectionCandidateToDifferentialMatcher(
    const Addr index_pc, const Addr target_pc
)
{
    bool successfully_added = differential_matcher.addCandidate(
        index_pc, target_pc
    );
    // TODO: Handle the case when we cannot add the candidate
    if (!successfully_added) {
        DMP_PREFETCHER_DEBUG(
            "Failed to add indirection candidate to Differential Matcher: "
            "Index PC %#x, Target PC %#x\n",
            index_pc, target_pc
        );
        return;
    }
    DMP_PREFETCHER_DEBUG(
        "Adding indirection candidate to Differential Matcher: Index PC %#x, "
        "Target PC %#x\n",
        index_pc, target_pc
    );
}

void
DifferentialMatchingPrefetcher::handleNewlyDetectedStride(const Addr pc)
{
    DMP_PREFETCHER_DEBUG(
        "New stride detected: PC %#x\n", pc
    );
    index_queue.add(pc, curTick());
    promoteIndexPcFromIqToIcs();
}

void
DifferentialMatchingPrefetcher::handleIcsHasAvailableSlots()
{
    DMP_PREFETCHER_DEBUG(
        "ICS has available slots. Try promoting index PCs from IQ to ICS.\n"
    );
    promoteIndexPcFromIqToIcs();
}

void
DifferentialMatchingPrefetcher::handleNewCandidateFromIcs(
    const Addr index_pc, const Addr target_pc
)
{
    DMP_PREFETCHER_DEBUG(
        "New candidate pair promoted: Index PC %#x, Target PC %#x\n",
        index_pc, target_pc
    );
    if (indirect_relation_table.containsEntry(index_pc, target_pc)) {
        DMP_PREFETCHER_DEBUG(
            "Candidate pair already exists in Indirect Relation Table: "
            "Index PC %#x, Target PC %#x\n",
            index_pc, target_pc
        );
        return;
    }
    addIndirectionCandidateToDifferentialMatcher(index_pc, target_pc);
}

void
DifferentialMatchingPrefetcher::handleDifferentialMatchResult(
    const Addr index_pc, const Addr target_pc, const bool successful_match,
    const Addr target_base_vaddr, const int64_t shift_amount,
    const AccessType index_access_type, const AccessType target_access_type
)
{
    DMP_PREFETCHER_DEBUG(
        "Differential match result for candidate pair: Index PC %#x, "
        "Target PC %#x, Successful Match %d\n",
        index_pc, target_pc, successful_match
    );
    if (!successful_match) {
        // Tell ICS that this candidate pair was unsuccessful so the ICS
        // records this unsuccessful attempt
        indirection_candidate_scoreboard.
            markPreviouslyUnsuccessfulMatch(index_pc, target_pc);
    } else {
        // Tell IQ that this index PC has a successful match and add th
        // target_pc to IQ
        index_queue.add(target_pc, curTick());
        // Tell ICS that this candidate pair was successful so the ICS clears
        // any negative history about this pair
        indirection_candidate_scoreboard.
            markPreviouslySuccessfulMatch(index_pc, target_pc);
        // Tell Indirect Relation Table about this successful match so that
        // it can be used for future prefetches
        indirect_relation_table.addEntry(
            index_pc, target_pc, target_base_vaddr, shift_amount,
            index_access_type, target_access_type
        );
    }
};

//void
//DifferentialMatchingPrefetcher::regProbeListeners()
//{
//    typedef ProbeListenerArg<
//        DifferentialMatchingPrefetcher, SimpleCacheAccessProbeArg
//    > DataAccessListener;
//    ProbeManager *pm = l1_controller->getProbeManager();
//    listeners.push_back(new DataAccessListener(
//        this,
//        "DataMovementHit",
//        &DifferentialMatchingPrefetcher::observeL1CacheHit
//    ));
//    pm->addListener("DataMovementHit", *(listeners.back()));
//
//    listeners.push_back(new DataAccessListener(
//        this,
//        "DataMovementMiss",
//        &DifferentialMatchingPrefetcher::observeL1CacheMiss
//    ));
//    pm->addListener("DataMovementMiss", *(listeners.back()));
//
//    listeners.push_back(new DataAccessListener(
//        this,
//        "DataMovementWriteback",
//        &DifferentialMatchingPrefetcher::observeL1CacheFill
//    ));
//    pm->addListener("DataMovementWriteback", *(listeners.back()));
//}

void
DifferentialMatchingPrefetcher::regStats()
{
    ProbeListenerObject::regStats();
}

void
DifferentialMatchingPrefetcher::addEventProbe(
    SimObject *obj, const char *event_name
)
{
    ProbeManager *pm = obj->getProbeManager();
    if (strcmp(event_name, "cpu outgoing data request") == 0) {
        listeners.push_back(new CpuRequestListener(this, pm, event_name));
        pm->addListener(event_name, *(listeners.back()));
    } else if (strcmp(event_name, "cpu incoming data response") == 0) {
        listeners.push_back(new CpuResponseListener(this, pm, event_name));
        pm->addListener(event_name, *(listeners.back()));
    } else if (strcmp(event_name, "DataMovementHit") == 0) {
        listeners.push_back(new CacheAccessListener(
            this, pm, event_name, /*is_hit*/ true, /*is_miss*/ false,
            /*is_fill*/ false
        ));
        pm->addListener(event_name, *(listeners.back()));
    } else if (strcmp(event_name, "DataMovementMiss") == 0) {
        listeners.push_back(new CacheAccessListener(
            this, pm, event_name, /*is_hit*/ false, /*is_miss*/ true,
            /*is_fill*/ false
        ));
        pm->addListener(event_name, *(listeners.back()));
    } else if (strcmp(event_name, "DataMovementWriteback") == 0) {
        listeners.push_back(new CacheAccessListener(
            this, pm, event_name, /*is_hit*/ false, /*is_miss*/ false,
            /*is_fill*/ true
        ));
        pm->addListener(event_name, *(listeners.back()));
    } else {
        panic("Unsupported event name for DMP prefetcher: %s", event_name);
    }
}

std::string
DifferentialMatchingPrefetcher::getPrefetcherName() const
{
    return this->name();
}

bool
DifferentialMatchingPrefetcher::isObservable(
    const SimpleCacheAccessProbeArg &arg
)
{
    const bool has_vaddr = arg.req->hasVaddr();
    const bool has_pc = arg.req->hasPC();
    const bool is_uncacheable = arg.req->isUncacheable();
    const bool is_instruction = arg.req->isInstFetch();

    // https://developer.arm.com/documentation/101811/0105/Address-spaces/
    // Size-of-virtual-addresses
    //const bool is_in_kernel_address_space = (has_vaddr && has_pc) ?
    //    (bits(arg.req->getVaddr(), 63, 48) == 0xFFFF) : false;
    //if (is_in_kernel_address_space) {
    //    // We don't want to track kernel address accesses.
    //    return false;
    //}

    // We only want to observe data cache accesses that,
    // - have virtual address
    // - have PC (so we can track them in the matcher)
    // - not be uncacheable, e.g., I/O accesses
    // - not be instruction fetches, as we are doing data prefetching
    if (has_vaddr && has_pc && !is_uncacheable && !is_instruction) {
        return true;
    }
    return false;
}

void
DifferentialMatchingPrefetcher::observeL1CacheHit(
    const SimpleCacheAccessProbeArg &arg
)
{
    if (!isObservable(arg)) {
        return;
    }

    if (!arg.hasCacheFillData()) {
        // We only care about cache hits with data
        warn("DMP Prefetcher observed L1 cache hit without data");
        return;
    }

    stats.numPrefetchableL1CacheHits++;

    DMP_CACHE_OBSERVER_DEBUG(
        "DMP L1 Cache HIT observed: paddr=%#x, vaddr=%#x, size=%d, pc=%#x, "
        "hasData=%d\n",
        arg.req->getPaddr(), arg.req->getVaddr(), arg.req->getSize(),
        arg.req->getPC(), arg.hasCacheFillData()
    );

    const Addr pc = arg.req->getPC();
    const uint64_t access_size = arg.req->getSize();
    const Addr paddr = arg.req->getPaddr();
    const Tick access_timestamp = curTick();
    stride_tracker.track(pc, access_size, paddr, access_timestamp);
    //if (!differential_matcher.isEmpty()) {
    //    differential_matcher.trackL1CacheHit(
    //        pc,
    //        arg.req->getVaddr(), // matcher tracks effective virtual address
    //        getDataFromProbe(arg),
    //        arg.req->getSize()
    //    );
    //}
    indirect_relation_table.trackL1CacheAccess(
        arg.req->getPC(),
        arg.req->getVaddr(),
        arg.req->getSize()
    );
}

void
DifferentialMatchingPrefetcher::observeL1CacheMiss(
    const SimpleCacheAccessProbeArg &arg
)
{
    if (!isObservable(arg)) {
        return;
    }

    stats.numPrefetchableL1CacheMisses++;

    DMP_CACHE_OBSERVER_DEBUG(
        "L1 Cache MISS observed: paddr=%#x, vaddr=%#x, size=%d, pc=%#x, "
        "hasData=%d\n",
        arg.req->getPaddr(), arg.req->getVaddr(), arg.req->getSize(),
        arg.req->getPC(), arg.hasCacheFillData()
    );
    const Addr pc = arg.req->getPC();
    const uint64_t access_size = arg.req->getSize();
    const Addr paddr = arg.req->getPaddr();
    const Tick access_timestamp = curTick();
    stride_tracker.track(pc, access_size, paddr, access_timestamp);
    indirection_candidate_scoreboard.trackL1CacheMiss(pc);
    //if (!differential_matcher.isEmpty()) {
    //    differential_matcher.trackL1CacheMiss(
    //        pc,
    //        arg.req->getVaddr(), // matcher tracks effective virtual address
    //        arg.req->getSize()
    //    );
    //}
    indirect_relation_table.trackL1CacheAccess(
        arg.req->getPC(),
        arg.req->getVaddr(),
        arg.req->getSize()
    );
}

void
DifferentialMatchingPrefetcher::observeL1CacheFill(
    const SimpleCacheAccessProbeArg &arg
)
{
    if (!isObservable(arg)) {
        return;
    }

    if (!arg.hasCacheFillData()) {
        // We only care about cache fills with data
        warn("DMP Prefetcher observed L1 cache fill without data");
        return;
    }

    DMP_CACHE_OBSERVER_DEBUG(
        "L1 Cache FILL observed: paddr=%#x, vaddr=%#x, size=%d, pc=%#x, "
        "hasData=%d\n",
        arg.req->getPaddr(), arg.req->getVaddr(), arg.req->getSize(),
        arg.req->getPC(), arg.hasCacheFillData()
    );

    //if (!differential_matcher.isEmpty()) {
    //    differential_matcher.trackL1CacheFill(
    //        arg.req->getPC(),
    //        arg.req->getVaddr(), // matcher tracks effective virtual address
    //        getDataFromProbe(arg),
    //        arg.req->getSize()
    //    );
//
    //}
}

void
DifferentialMatchingPrefetcher::observeOutgoingCpuRequest(const RequestPtr req)
{
    const bool has_vaddr = req->hasVaddr();
    const bool has_pc = req->hasPC();
    const bool is_uncacheable = req->isUncacheable();
    const bool is_instruction = req->isInstFetch();

    DMP_CACHE_OBSERVER_DEBUG(
        "CPU request observed: paddr=%#x, vaddr=%#x, size=%d, pc=%#x\n",
        req->getPaddr(), req->getVaddr(), req->getSize(), req->getPC()
    );

    if (!has_vaddr || !has_pc || is_uncacheable || is_instruction) {
        // We only want to observe data cache accesses that,
        // - have virtual address
        // - have PC (so we can track them in the matcher)
        // - not be uncacheable, e.g., I/O accesses
        // - not be instruction fetches, as we are doing data prefetching
        return;
    }

    if (!differential_matcher.isEmpty()) {
        differential_matcher.trackCpuOutgoingRequest(
            req->getPC(),
            req->getVaddr(),
            req->getSize()
        );
    }
}

void
DifferentialMatchingPrefetcher::observeIncomingCpuResponse(
    const PacketPtr pkt
)
{
    const bool has_vaddr = req->hasVaddr();
    const bool has_pc = req->hasPC();
    const bool is_uncacheable = req->isUncacheable();
    const bool is_instruction = req->isInstFetch();

    DMP_CACHE_OBSERVER_DEBUG(
        "CPU response observed: paddr=%#x, vaddr=%#x, size=%d, pc=%#x\n",
        req->getPaddr(), req->getVaddr(), req->getSize(), req->getPC()
    );

    if (!has_vaddr || !has_pc || is_uncacheable || is_instruction) {
        // We only want to observe data cache accesses that,
        // - have virtual address
        // - have PC (so we can track them in the matcher)
        // - not be uncacheable, e.g., I/O accesses
        // - not be instruction fetches, as we are doing data prefetching
        return;
    }

    uint64_t data = 0;
    const uint8_t* data_ptr = pkt->getConstPtr<uint8_t>();
    for (unsigned i = 0; i < req->getSize(); ++i) {
        data |= static_cast<uint64_t>(data_ptr[i]) << (i*8);
    }

    if (!differential_matcher.isEmpty()) {
        differential_matcher.trackCpuIncomingResponse(
            req->getPC(),
            req->getVaddr(),
            data,
            req->getSize()
        );
    }
}

void
DifferentialMatchingPrefetcher::notifyNewPrefetchRequest(
    const enums::CacheLevel cache_controller_level
)
{
    if (cache_controller_level == enums::CacheLevel::L1) {
        l1_controller->notifyPrefetcherProxyOfNewPrefetchRequest();
    } else if (cache_controller_level == enums::CacheLevel::L2) {
        l2_controller->notifyPrefetcherProxyOfNewPrefetchRequest();
    } else {
        panic("Unknown cache controller level in CacheLevel");
    }
}

void
DifferentialMatchingPrefetcher::handleNewPrefetchedDataFromStridePrefetcher(
    const Addr target_paddr, const Addr pc, const uint64_t data
)
{
    DMP_PREFETCHER_DEBUG(
        "Received new prefetched data from stride prefetcher: "
        "target_paddr=%#x, pc=%#x, data=%#x\n",
        target_paddr, pc, data
    );
    std::optional<std::vector<PrefetchRequest>> new_prefetches =
        indirect_relation_table.queryEntryByIndexPc(
            /*index_pc*/ pc,
            /*data_from_index_pc*/ data
        );
    if (new_prefetches.has_value()) {
        stats.numStridePrefetchesSentToIRT++;
        for (const PrefetchRequest &new_prefetch : new_prefetches.value()) {
            DMP_PREFETCHER_DEBUG(
                "New prefetch generated from stride prefetcher data: "
                "target_paddr=%#x, pc=%#x, new_prefetch=%#x\n",
                target_paddr, pc, new_prefetch.prefetch_vaddr
            );
            if (enable_dmp_prefetching) {
                // We don't need to check for memory bounds here as we're
                // working with virtual addresses.
                dmp_prefetch_queue->enqueuePendingRequest(new_prefetch);
            }
        }
    }
}

bool
DifferentialMatchingPrefetcher::isATargetPC(const Addr pc) const
{
    return indirect_relation_table.isATargetPC(pc);
}

Addr
DifferentialMatchingPrefetcher::getBlockAddress(Addr addr) const
{
    return addr & ~((Addr)cache_line_size-1);
}

uint64_t
DifferentialMatchingPrefetcher::getDataFromProbe(
    const SimpleCacheAccessProbeArg &arg
) const
{
    uint64_t pkt_data = 0;
    const uint8_t* pkt_data_ptr = arg.cache_fill_data.data();
    for (unsigned i = 0; i < arg.req->getSize(); ++i) {
        pkt_data |= static_cast<uint64_t>(pkt_data_ptr[i]) << (i*8);
    }
    return pkt_data;
}

PrefetcherStats&
DifferentialMatchingPrefetcher::getStats()
{
    return stats;
}

} // namespace dmp

} // namespace prefetch

} // namespace gem5
