/*
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

#include "pickle/application_specific/prefetcher/prefetcher_work_tracker.hh"

#include "debug/PickleDevicePrefetcherTrace.hh"
#include "debug/PickleDevicePrefetcherWorkTrackerDebug.hh"
#include "debug/PickleDevicePrefetcherWorkTrackerStatsDebug.hh"
#include "pickle/device/pickle_device.hh"

namespace gem5
{

PrefetcherWorkTracker::PrefetcherWorkTracker()
  : job_id(-1ULL),
    core_id(-1ULL),
    is_activated(false),
    owner(nullptr),
    job_descriptor(nullptr),
    software_hint_distance(0),
    hardware_prefetch_distance(0),
    current_core_work_item(-1ULL)
{
}

PrefetcherWorkTracker::PrefetcherWorkTracker(
    PicklePrefetcher* owner,
    const uint64_t _job_id, const uint64_t _core_id,
    std::shared_ptr<PickleJobDescriptor> _job_descriptor
) : job_id(_job_id),
    core_id(_core_id),
    is_activated(true),
    owner(owner),
    job_descriptor(_job_descriptor),
    current_core_work_item(-1ULL)
{
    software_hint_distance = owner->getSoftwareHintPrefetchDistance();
    hardware_prefetch_distance = \
        owner->getSoftwareHintPrefetchDistance() - \
        owner->getPrefetchDistanceOffsetFromSoftwareHint();

    if (job_descriptor->kernel_name == "bfs_kernel") {
        prefetch_generator = std::make_shared<BFSPrefetchGenerator>(
            "BFSPrefetchGenerator",
            owner->getPrefetchDistanceOffsetFromSoftwareHint(),
            this
        );
    } else if (job_descriptor->kernel_name == "pr_kernel") {
        prefetch_generator = std::make_shared<PRPrefetchGenerator>(
            "PRPrefetchGenerator",
            owner->getPrefetchDistanceOffsetFromSoftwareHint(),
            this
        );
    } else {
        panic(
            "Unknown prefetch generator mode: %s\n",
            job_descriptor->kernel_name
        );
    }
}

void
PrefetcherWorkTracker::addWorkItem(Addr work_id)
{
    if (!is_activated) {
        return;
    }
    auto workItem = prefetch_generator->generateWorkItem(work_id);
    workItem->setJobId(job_id);
    work_id_to_work_items_map[work_id] = workItem;
    populateCurrLevelPrefetches(workItem);
    if (job_descriptor->kernel_name == "bfs_kernel") {
        notifyCoreCurrentWork(work_id - software_hint_distance * 4);
    } else if (job_descriptor->kernel_name == "pr_kernel") {
        notifyCoreCurrentWork(work_id - software_hint_distance);
    } else {
        panic(
            "Unknown prefetch generator mode: %s\n",
            job_descriptor->kernel_name
        );
    }
}

void
PrefetcherWorkTracker::processIncomingPrefetch(const Addr pf_vaddr)
{
    std::vector<std::shared_ptr<WorkItem>>& work_items_induced_pf_vaddr = \
        pf_vaddr_to_work_items_map[pf_vaddr];
    std::vector<std::shared_ptr<WorkItem>> work_items_have_more_requests;
    for (auto &work: work_items_induced_pf_vaddr) {
        work->removeExpectedPrefetch(pf_vaddr);
        DPRINTF(
            PickleDevicePrefetcherWorkTrackerDebug,
            "WorkItem 0x%llx receives pf 0x%llx\n",
            work->getWorkId(), pf_vaddr
        );
        if (work->isDoneWithCurrLevel()) {
            work->moveToNextLevel();
            DPRINTF(
                PickleDevicePrefetcherWorkTrackerDebug,
                "WorkItem 0x%llx moves to level %lld\n",
                work->getWorkId(), work->getLevel()
            );
            if (work->isDone()) {
                // profile the work
                profileWork(work);
                // if the core has not worked on this work item, we keep
                // the Tick when the work item has finished
                if (!(work->hasCoreWorkedOnThisWork())) {
                    pf_complete_time[work->getWorkId()] = \
                        work->getPrefetchCompleteTime();
                    assert(work->getPrefetchCompleteTime() != 0);
                }
                // remove the work
                work_id_to_work_items_map.erase(work->getWorkId());
                continue;
            }
            work_items_have_more_requests.push_back(work);
        }
    }
    pf_vaddr_to_work_items_map[pf_vaddr].clear();
    for (auto work: work_items_have_more_requests) {
        populateCurrLevelPrefetches(work);
    }
    if (work_items_have_more_requests.size() > 0) {
        DPRINTF(
            PickleDevicePrefetcherWorkTrackerDebug,
            "Scheduled out queue\n"
        );
        owner->scheduleDueToNewOutstandingPrefetchRequests();
    }
}

void
PrefetcherWorkTracker::populateCurrLevelPrefetches(
    std::shared_ptr<WorkItem> work
)
{
    for (auto addr: work->getCurrLevelExpectedPrefetches()) {
        outstanding_prefetches.emplace(
            addr, work->getWorkItemReceiveTime()
        );
        if (
            pf_vaddr_to_work_items_map.find(addr) \
                == pf_vaddr_to_work_items_map.end()
        ) {
            pf_vaddr_to_work_items_map[addr] = \
                std::vector<std::shared_ptr<WorkItem>>();
        }
        pf_vaddr_to_work_items_map[addr].push_back(work);
        DPRINTF(
            PickleDevicePrefetcherWorkTrackerDebug,
            "Adding pf_vaddr 0x%llx from WorkItem = 0x%llx\n",
            addr, work->getWorkId()
        );
    }
}

bool
PrefetcherWorkTracker::hasOutstandingPrefetch() const
{
    return !(outstanding_prefetches.empty());
}

PrefetchRequest
PrefetcherWorkTracker::peekNextPrefetch() const
{
    return outstanding_prefetches.top();
}

void
PrefetcherWorkTracker::popPrefetch()
{
    outstanding_prefetches.pop();
}

void
PrefetcherWorkTracker::profileWork(std::shared_ptr<WorkItem> work)
{
    owner->profileWork(work, job_id, core_id);
}

void
PrefetcherWorkTracker::notifyCoreCurrentWork(const Addr work_id)
{
    // If the prefetch for this work is complete (timely prefetch), we have the
    // prefetch complete time in pf_complete_time. It's possible that the core
    // has already worked on this work item but never sent a prefetch request.
    // If the prefetch for this work is not done (late prefetch), we profile
    // the core access time. Note that, it's possible that this work has never
    // been requested by the core.
    auto it = pf_complete_time.find(work_id);
    DPRINTF(
        PickleDevicePrefetcherWorkTrackerDebug,
        "notifyCoreCurrentWork: core_id: %lld, work_id 0x%llx\n",
        core_id, work_id
    );
    if (it != pf_complete_time.end()) {
        const Tick complete_time = it->second;
        owner->profileTimelyPrefetch(
            complete_time, job_id, core_id
        );
        pf_complete_time.erase(it);
        DPRINTF(
            PickleDevicePrefetcherWorkTrackerDebug,
            "notifyCoreCurrentWork: Found pf_complete_time = %lld\n",
            complete_time
        );
    } else {
        if (
            work_id_to_work_items_map.find(work_id) != \
                work_id_to_work_items_map.end()
        ) {
            auto work_item = \
                work_id_to_work_items_map[work_id];
            work_item->notifyCoreIsWorkingOnThisWork();
            DPRINTF(
                PickleDevicePrefetcherWorkTrackerDebug,
                "notifyCoreCurrentWork: Notified work_item\n"
            );
        }
        else
        {
            DPRINTF(
                PickleDevicePrefetcherWorkTrackerDebug,
                "notifyCoreCurrentWork: No work item found\n"
            );
        }
    }
}

}; // namespace gem5
