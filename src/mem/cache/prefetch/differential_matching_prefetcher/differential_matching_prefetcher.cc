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

#include "base/intmath.hh"
#include "base/logging.hh"
#include "base/random.hh"
#include "base/trace.hh"
#include "debug/DifferentialMatchingPrefetcherCacheObserverDebug.hh"
#include "debug/DifferentialMatchingPrefetcherDebug.hh"
#include "mem/cache/cache_probe_arg.hh"
#include "params/DifferentialMatchingPrefetcher.hh"
#include "sim/clocked_object.hh"
#include "sim/probe/probe.hh"

namespace gem5
{

namespace prefetch
{

DifferentialMatchingPrefetcher::DifferentialMatchingPrefetcher(
    const DifferentialMatchingPrefetcherParams &p
) : ProbeListenerObject(p), system(p.system),
    cache_line_size(p.system->cacheLineSize()),
    l1_controller(p.l1_controller),
    process_detection_event(
        [this]{processDetectionEvent();}, name() + ".process_detection_event"
    ),
    index_queue_size(p.index_queue_size),
    indirection_candidate_scoreboard_num_entries(
        p.indirection_candidate_scoreboard_num_entries
    ),
    indirection_candidate_scoreboard_num_candidates_per_entry(
        p.indirection_candidate_scoreboard_num_candidates_per_entry
    ),
    sample_window_size(p.sample_window_size),
    stride_tracker(
        /*capacity*/ p.index_queue_size,
        /*_confidence_threshold*/ 0.5,
        /*_cache_block_size*/ p.system->cacheLineSize(),
        /*_prefetcher_interface*/ this
    ),
    index_queue(p.index_queue_size, IndexQueueReplacementPolicy::LowestScore),
    indirection_candidate_scoreboard(
        /*_max_num_entries*/ p.indirection_candidate_scoreboard_num_entries,
        /*_max_num_candidates*/
        p.indirection_candidate_scoreboard_num_candidates_per_entry,
        /*_sample_window_size*/
        p.sample_window_size,
        /*_prefetcher_interface*/
        this
    )
{
    panic_if(l1_controller == nullptr,
            "L1 controller pointer passed to DMP prefetcher is null");
}

void
DifferentialMatchingPrefetcher::processDetectionEvent()
{
    // Here, we move candidate PC around the components
    promoteIndexPcFromIqToIcs(); // IQ -> ICS
}

void
DifferentialMatchingPrefetcher::scheduleHandleDetectionEvent()
{
    if (!process_detection_event.scheduled()) {
        schedule(
            process_detection_event,
            curTick() + 250 // 250 ticks = 1 clock cycle at 4GHz
        );
    }
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
                index_queue.increaseTriedCount(pc);
                DMP_PREFETCHER_DEBUG(
                    "Index PC %#x promoted from IQ to ICS\n", pc
                );
                break;
            }
        }
    }
}

void
DifferentialMatchingPrefetcher::\
    addIndirectionCandidateToDifferentialMatcher(
    const Addr index_pc, const Addr target_pc
)
{
    // TODO: implement the differential matching logic here
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
    scheduleHandleDetectionEvent();
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
    // TODO: Start differential matching for this pair of PCs
}

void
DifferentialMatchingPrefetcher::regProbeListeners()
{
    typedef ProbeListenerArg<
        DifferentialMatchingPrefetcher, SimpleCacheAccessProbeArg
    > DataAccessListener;
    ProbeManager *pm = l1_controller->getProbeManager();
    listeners.push_back(new DataAccessListener(
        this,
        "DataMovementHit",
        &DifferentialMatchingPrefetcher::observeL1CacheHit
    ));
    pm->addListener("DataMovementHit", *(listeners.back()));

    listeners.push_back(new DataAccessListener(
        this,
        "DataMovementMiss",
        &DifferentialMatchingPrefetcher::observeL1CacheMiss
    ));
    pm->addListener("DataMovementMiss", *(listeners.back()));

    listeners.push_back(new DataAccessListener(
        this,
        "DataMovementWriteback",
        &DifferentialMatchingPrefetcher::observeL1CacheFill
    ));
    pm->addListener("DataMovementWriteback", *(listeners.back()));
}

bool
DifferentialMatchingPrefetcher::isObservable(
    const SimpleCacheAccessProbeArg &arg
)
{
    // We only observe data access with virtual address
    return arg.req->hasVaddr();
}

void
DifferentialMatchingPrefetcher::observeL1CacheHit(
    const SimpleCacheAccessProbeArg &arg
)
{
    if (!isObservable(arg)) {
        return;
    }

    DMP_CACHE_OBSERVER_DEBUG(
        "DMP L1 Cache HIT observed: paddr=%#x, vaddr=%#x, size=%d, pc=%#x, "
        "hasData=%d\n",
        arg.req->getPaddr(), arg.req->getVaddr(), arg.req->getSize(),
        arg.req->getPC(), arg.hasCacheFillData()
    );

    const Addr pc = arg.req->getPC();
    const Addr block_address = getBlockAddress(arg.req->getPaddr());
    const Tick access_timestamp = curTick();
    stride_tracker.track(pc, block_address, access_timestamp);
}

void
DifferentialMatchingPrefetcher::observeL1CacheMiss(
    const SimpleCacheAccessProbeArg &arg
)
{
    if (!isObservable(arg)) {
        return;
    }

    DMP_CACHE_OBSERVER_DEBUG(
        "DMP L1 Cache MISS observed: paddr=%#x, vaddr=%#x, size=%d, pc=%#x, "
        "hasData=%d\n",
        arg.req->getPaddr(), arg.req->getVaddr(), arg.req->getSize(),
        arg.req->getPC(), arg.hasCacheFillData()
    );
    const Addr pc = arg.req->getPC();
    const Addr block_address = getBlockAddress(arg.req->getPaddr());
    const Tick access_timestamp = curTick();
    stride_tracker.track(pc, block_address, access_timestamp);
    indirection_candidate_scoreboard.trackL1CacheMiss(pc);
}

void
DifferentialMatchingPrefetcher::observeL1CacheFill(
    const SimpleCacheAccessProbeArg &arg
)
{
    if (!isObservable(arg)) {
        return;
    }

    DMP_CACHE_OBSERVER_DEBUG(
        "DMP L1 Cache FILL observed: paddr=%#x, vaddr=%#x, size=%d, pc=%#x, "
        "hasData=%d\n",
        arg.req->getPaddr(), arg.req->getVaddr(), arg.req->getSize(),
        arg.req->getPC(), arg.hasCacheFillData()
    );
}

Addr
DifferentialMatchingPrefetcher::getBlockAddress(Addr addr) const
{
    return addr & ~((Addr)cache_line_size-1);
}

} // namespace prefetch
} // namespace gem5
