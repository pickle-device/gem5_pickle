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

#include "base/logging.hh"
#include "debug/PickleDevicePrefetcherTrace.hh"
#include "debug/PickleDevicePrefetcherWorkTrackerDebug.hh"
#include "debug/PickleDevicePrefetcherWorkTrackerStatsDebug.hh"
#include "pickle/application_specific/prefetcher/pickle_prefetcher.hh"
#include "pickle/device/pickle_device.hh"

namespace gem5
{

PrefetcherWorkTracker::PrefetcherWorkTracker()
  : job_id(-1ULL),
    core_id(-1ULL),
    is_activated(false),
    prefetch_dropping_distance(0),
    owner(nullptr),
    collective(nullptr),
    job_descriptor(nullptr),
    software_hint_distance(0),
    hardware_prefetch_distance(0)
{
}

PrefetcherWorkTracker::PrefetcherWorkTracker(
    PicklePrefetcher* owner,
    std::shared_ptr<PrefetcherWorkTrackerCollective> _collective,
    const uint64_t _job_id, const uint64_t _core_id,
    std::shared_ptr<PickleJobDescriptor> _job_descriptor,
    const uint64_t _prefetch_dropping_distance
) : job_id(_job_id),
    core_id(_core_id),
    is_activated(true),
    enable_dropping_prefetches(_prefetch_dropping_distance > 0),
    prefetch_dropping_distance(_prefetch_dropping_distance),
    owner(owner),
    collective(_collective),
    job_descriptor(_job_descriptor)
{
    software_hint_distance = owner->getSoftwareHintPrefetchDistance();
    hardware_prefetch_distance = \
        owner->getSoftwareHintPrefetchDistance() - \
        owner->getPrefetchDistanceOffsetFromSoftwareHint();

    if (job_descriptor->kernel_name == "bfs_kernel") {
        prefetch_generator = std::make_shared<BFSPrefetchGenerator>(
            "BFSPrefetchGenerator",
            owner->getSoftwareHintPrefetchDistance(),
            owner->getPrefetchDistanceOffsetFromSoftwareHint(),
            this
        );
    } else if (job_descriptor->kernel_name == "pr_kernel") {
        prefetch_generator = std::make_shared<PRPrefetchGenerator>(
            "PRPrefetchGenerator",
            owner->getSoftwareHintPrefetchDistance(),
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
PrefetcherWorkTracker::addWorkItem(Addr work_data)
{
    if (!is_activated) {
        return;
    }
    // work_data is the current node that the core is working on.
    // The prefetcher generator will need to figure out the next node to
    // prefetch.
    auto work_item = prefetch_generator->generateWorkItem(work_data);
    work_item->setJobId(job_id);
    work_item->setCoreId(core_id);
    work_id_to_work_items_map[work_item->getWorkId()] = work_item;
    DPRINTF(
        PickleDevicePrefetcherWorkTrackerDebug,
        "addWorkItem: job_id: %lld, core_id: %lld, work_id 0x%llx\n",
        job_id, core_id, work_item->getWorkId()
    );
    pending_work_items.push(work_item);
    tryNotifyCoreCurrentWork(work_data);
}

bool
PrefetcherWorkTracker::hasPendingWorkItem() const
{
    return !(pending_work_items.empty());
}

std::shared_ptr<WorkItem>
PrefetcherWorkTracker::peekNextWorkItem() const
{
    return pending_work_items.top();
}
void
PrefetcherWorkTracker::popWorkItem()
{
    pending_work_items.pop();
}

void
PrefetcherWorkTracker::profileWork(std::shared_ptr<WorkItem> work)
{
    owner->profileWork(work, job_id, core_id);
}

void
PrefetcherWorkTracker::tryNotifyCoreCurrentWork(const Addr work_id)
{
    // If the prefetch for this work is complete (timely prefetch), we have the
    // prefetch complete time in pf_complete_time_map. It's possible that the
    // core has already worked on this work item but never sent a prefetch
    // request.
    // If the prefetch for this work is not done (late prefetch), we profile
    // the core access time. Note that, it's possible that this work has never
    // been requested by the core.
    auto it = collective->pf_complete_time_map[job_id].find(work_id);
    DPRINTF(
        PickleDevicePrefetcherWorkTrackerDebug,
        "tryNotifyCoreCurrentWork: core_id: %lld, job_id: %lld, "
        "work_id 0x%llx\n",
        core_id, job_id, work_id
    );
    if (it != collective->pf_complete_time_map[job_id].end()) {
        const Tick complete_time = it->second;
        owner->profileTimelyPrefetch(
            complete_time, curTick(), job_id, core_id
        );
        collective->pf_complete_time_map[job_id].erase(it);
        DPRINTF(
            PickleDevicePrefetcherWorkTrackerDebug,
            "tryNotifyCoreCurrentWork: Found pf_complete_time_map = %lld\n",
            complete_time
        );
    } else {
        collective->setCoreStartTime(job_id, work_id, curTick());
        DPRINTF(
            PickleDevicePrefetcherWorkTrackerDebug,
            "tryNotifyCoreCurrentWork: Added to core_start_time_map\n"
        );
    }
    setCoreLatestWorkId(work_id);
    DPRINTF(
        PickleDevicePrefetcherWorkTrackerDebug,
        "tryNotifyCoreCurrentWork: core_latest_work_id = 0x%llx\n",
        getCoreLatestWorkId()
    );
}

void
PrefetcherWorkTracker::stopTrackingWork(const Addr work_id)
{
    auto it = work_id_to_work_items_map.find(work_id);
    if (it != work_id_to_work_items_map.end()) {
        work_id_to_work_items_map.erase(it);
    } else {
        DPRINTF(
            PickleDevicePrefetcherWorkTrackerDebug,
            "stopTrackingWork: No work item found\n"
        );
    }
}

void
PrefetcherWorkTracker::setCoreLatestWorkId(const Addr work_id)
{
    core_latest_work_id = work_id;
}

Addr
PrefetcherWorkTracker::getCoreLatestWorkId() const
{
    return core_latest_work_id;
}

void
PrefetcherWorkTracker::updateWorkItemQueue()
{
    // dropping prefetches that were worked on by the core
    if (enable_dropping_prefetches) {
        while (hasPendingWorkItem()) {
            std::shared_ptr<WorkItem> work_item = peekNextWorkItem();
            const uint64_t job_id = work_item->getJobId();
            const uint64_t work_id = work_item->getWorkId();
            if (collective->hasCoreWorkedOnThisWork(job_id, work_id)) {
                DPRINTF(
                    PickleDevicePrefetcherWorkTrackerDebug,
                    "Dropping work item 0x%llx, already worked on by the "
                    "core\n",
                    work_id
                );
                popWorkItem();
                collective->untrackCoreStartTime(job_id, work_id);
            } else {
                break;
            }
        }
    }
}

PrefetcherWorkTrackerCollective::PrefetcherWorkTrackerCollective()
  : max_active_work_items(0),
    owner(nullptr)
{
}

PrefetcherWorkTrackerCollective::PrefetcherWorkTrackerCollective(
    const uint64_t _max_active_work_items
) : max_active_work_items(_max_active_work_items),
    owner(nullptr)
{
}

void
PrefetcherWorkTrackerCollective::setOwner(PicklePrefetcher* owner)
{
    this->owner = owner;
}

void
PrefetcherWorkTrackerCollective::addPrefetcherWorkTracker(
    const uint64_t job_id, const uint64_t core_id,
    std::shared_ptr<PrefetcherWorkTracker> tracker
)
{
    if (pf_complete_time_map.find(job_id) == pf_complete_time_map.end()) {
        pf_complete_time_map[job_id] = std::unordered_map<uint64_t, Tick>();
    }
    if (core_start_time_map.find(job_id) == core_start_time_map.end()) {
        core_start_time_map[job_id] = std::unordered_map<uint64_t, Tick>();
    }
    trackers[std::make_pair(job_id, core_id)] = tracker;
}

std::shared_ptr<PrefetcherWorkTracker>
PrefetcherWorkTrackerCollective::getPrefetcherWorkTracker(
    const uint64_t job_id, const uint64_t core_id
)
{
    return trackers[std::make_pair(job_id, core_id)];
}

std::shared_ptr<WorkItem>
PrefetcherWorkTrackerCollective::getAndPopNextWorkItem()
{
    // TODO: replace this with a replacement policy object
    Tick min_time = -1ULL;
    std::shared_ptr<PrefetcherWorkTracker> min_time_tracker = nullptr;
    for (auto tracker: trackers) {
        std::shared_ptr<PrefetcherWorkTracker> tracker_ptr = tracker.second;
        if (tracker_ptr->hasPendingWorkItem()) {
            std::shared_ptr<WorkItem> work_item =
                tracker_ptr->peekNextWorkItem();
            if (work_item->getWorkItemReceiveTime() < min_time) {
                min_time = work_item->getWorkItemReceiveTime();
                min_time_tracker = tracker_ptr;
            }
        }
    }
    if (min_time_tracker == nullptr) {
        return nullptr;
    }
    std::shared_ptr<WorkItem> work_item = min_time_tracker->peekNextWorkItem();
    min_time_tracker->popWorkItem();
    return work_item;
}

bool
PrefetcherWorkTrackerCollective::hasOutstandingPrefetchRequest() const
{
    return !(outstanding_prefetch_queue.empty());
}

PrefetchRequest
PrefetcherWorkTrackerCollective::peekNextPrefetchRequest() const
{
    return outstanding_prefetch_queue.top();
}

void
PrefetcherWorkTrackerCollective::popPrefetchRequest()
{
    outstanding_prefetch_queue.pop();
}

void
PrefetcherWorkTrackerCollective::processIncomingPrefetch(const Addr pf_vaddr)
{
    // if the work is in the pf_vaddr_to_work_items_map, it should be in the
    // active work items list
    std::vector<std::shared_ptr<WorkItem>>& work_items_induced_pf_vaddr = \
        pf_vaddr_to_work_items_map[pf_vaddr];
    std::vector<std::shared_ptr<WorkItem>> work_items_have_more_requests;
    bool there_is_a_work_item_done = false;
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
                owner->profileWork(work, work->getJobId(), work->getCoreId());
                there_is_a_work_item_done = true;
                // if the core has not worked on this work item, we keep
                // the Tick when the work item has finished
                if (!hasCoreWorkedOnThisWork(
                    work->getJobId(), work->getWorkId()
                )) {
                    profilePrefetchCompleteTime(
                        work->getJobId(), work->getWorkId(),
                        work->getPrefetchCompleteTime()
                    );
                    DPRINTF(
                        PickleDevicePrefetcherWorkTrackerDebug,
                        "WorkItem 0x%llx (job_id %lld) has been added to "
                        "pf_complete_time_map \n",
                        work->getWorkId(), work->getJobId()
                    );
                    assert(work->getPrefetchCompleteTime() != 0);
                }
                // remove the work
                getPrefetcherWorkTracker(
                    work->getJobId(), work->getCoreId()
                )->stopTrackingWork(work->getWorkId());
                continue;
            }
            work_items_have_more_requests.push_back(work);
        }
    }
    pf_vaddr_to_work_items_map[pf_vaddr].clear();
    for (auto work: work_items_have_more_requests) {
        populateCurrLevelPrefetches(work);
    }
    if (there_is_a_work_item_done) {
        replaceActiveWorkItemsUponCompletion();
    }
    if (work_items_have_more_requests.size() > 0) {
        DPRINTF(
            PickleDevicePrefetcherWorkTrackerDebug,
            "Scheduled out queue\n"
        );
        owner->scheduleDueToOutstandingPrefetchRequests();
    }
}

void
PrefetcherWorkTrackerCollective::populateCurrLevelPrefetches(
    std::shared_ptr<WorkItem> work
)
{
    for (auto addr: work->getCurrLevelExpectedPrefetches()) {
        outstanding_prefetch_queue.emplace(
            addr, work->getWorkItemReceiveTime(), work->getWorkId()
        );
        if (
            pf_vaddr_to_work_items_map.find(addr) \
                == pf_vaddr_to_work_items_map.end()
        ) {
            pf_vaddr_to_work_items_map[addr] = \
                std::vector<std::shared_ptr<WorkItem>>();
            pf_vaddr_to_work_items_map[addr].reserve(2);
        }
        pf_vaddr_to_work_items_map[addr].push_back(work);
        DPRINTF(
            PickleDevicePrefetcherWorkTrackerDebug,
            "Adding pf_vaddr 0x%llx from WorkItem = 0x%llx\n",
            addr, work->getWorkId()
        );
    }
    owner->scheduleDueToOutstandingPrefetchRequests();
}

void
PrefetcherWorkTrackerCollective::replaceActiveWorkItemsUponCompletion()
{
    active_work_items.remove_if(
        [&](std::shared_ptr<WorkItem> item){
            if (item->isDone()) {
                DPRINTF(
                    PickleDevicePrefetcherWorkTrackerDebug,
                    "replaceActiveWorkItemUponCompletion: removing work_id "
                    "0x%llx\n",
                    item->getWorkId()
                );
            }
            return item->isDone();
        }
    );
    while (active_work_items.size() < max_active_work_items) {
        std::shared_ptr<WorkItem> work_item = getAndPopNextWorkItem();
        if (work_item == nullptr) {
            break;
        }
        work_item->profileWorkActivationTime();
        DPRINTF(
            PickleDevicePrefetcherWorkTrackerDebug,
            "replaceActiveWorkItemUponCompletion: adding work_id 0x%llx\n",
            work_item->getWorkId()
        );
        active_work_items.push_back(work_item);
        populateCurrLevelPrefetches(work_item);
    }
}

void
PrefetcherWorkTrackerCollective::profilePrefetchCompleteTime(
    const uint64_t job_id, const Addr work_id, const Tick complete_time
)
{
    pf_complete_time_map[job_id][work_id] = complete_time;
    DPRINTF(
        PickleDevicePrefetcherWorkTrackerDebug,
        "profilePrefetchCompleteTime: job_id: %lld, work_id 0x%llx\n",
        job_id, work_id
    );
}

Tick
PrefetcherWorkTrackerCollective::getCoreStartTime(
    const uint64_t job_id, const Addr work_id
)
{
    return core_start_time_map[job_id][work_id];
}

bool
PrefetcherWorkTrackerCollective::hasCoreWorkedOnThisWork(
    const uint64_t job_id, const Addr work_id
)
{
    return core_start_time_map[job_id].find(work_id) != \
        core_start_time_map[job_id].end();
}

void
PrefetcherWorkTrackerCollective::setCoreStartTime(
    const uint64_t job_id, const Addr work_id, const Tick start_time
)
{
    core_start_time_map[job_id][work_id] = start_time;
}

void
PrefetcherWorkTrackerCollective::untrackCoreStartTime(
    const uint64_t job_id, const Addr work_id
)
{
    core_start_time_map[job_id].erase(work_id);
}

}; // namespace gem5
