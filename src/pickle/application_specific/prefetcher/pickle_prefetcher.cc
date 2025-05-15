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

#include "pickle/application_specific/prefetcher/pickle_prefetcher.hh"

#include "debug/PickleDevicePrefetcherDebug.hh"
#include "debug/PickleDevicePrefetcherProgressTracker.hh"
#include "debug/PickleDevicePrefetcherTrace.hh"
#include "debug/PickleDevicePrefetcherWorkTrackerDebug.hh"
#include "pickle/device/pickle_device.hh"
#include "pickle/request_manager/manager.hh"

namespace gem5
{

PicklePrefetcher::PicklePrefetcher(
  const PicklePrefetcherParams &params
)
  : ClockedObject(params),
    software_hint_prefetch_distance(params.software_hint_prefetch_distance),
    prefetch_distance_offset_from_software_hint(
        params.prefetch_distance_offset_from_software_hint
    ),
    concurrent_work_item_capacity(
        params.concurrent_work_item_capacity
    ),
    expected_number_of_prefetch_generators(
        params.expected_number_of_prefetch_generators
    ),
    processInQueueEvent(
        [this]{processPrefetcherInQueue();},
        name() + ".operate_prefetcher_in_queue_event"
    ),
    processOutQueueEvent(
        [this]{processPrefetcherOutQueue();},
        name() + ".operate_prefetcher_out_queue_event"
    ),
    processGlobalOutstandingPrefetchQueueEvent(
        [this]{processGlobalOutstandingPrefetchQueue();},
        name() + ".operate_global_outstanding_prefetch_queue_event"
    ),
    replaceWorkItemsEvent(
        [this]{replaceWorkItem();},
        name() + ".replace_work_items_event"
    ),
    ticks_per_cycle(1000),
    num_cores(params.num_cores),
    prefetcher_initialized(false),
    owner(nullptr),
    workCount(0),
    prefetcherStats(this)
{
    panic_if(
        software_hint_prefetch_distance < \
            prefetch_distance_offset_from_software_hint,
        "Prefetch distance offset from software hint must not be greater "
        "than the prefetch distance\n"
    );

    panic_if(
        concurrent_work_item_capacity < 1,
        "The prefetcher must be able to handle at least 1 work item at a time"
        "\n"
    );

    for (
        int job_id = 0; job_id < expected_number_of_prefetch_generators;
        job_id++
    ) {
        taskStats.push_back(
            std::vector<std::shared_ptr<TaskStats>>()
        );
        taskStats.back().reserve(num_cores);
        for (int core_id = 0; core_id < num_cores; core_id++) {
            taskStats.back().push_back(
                std::shared_ptr<TaskStats>(
                    new TaskStats(this, job_id, core_id)
                )
            );
        }
    }
}

PicklePrefetcher::~PicklePrefetcher()
{
}

void
PicklePrefetcher::startup()
{
}

void
PicklePrefetcher::setOwner(PickleDevice* pickle_device)
{
    this->owner = pickle_device;
    this->ticks_per_cycle = pickle_device->getNumTicksPerCycle();
    panic_if(
        num_cores != pickle_device->getNumCores(),
        "The prefetcher and the pickle device must have the same number of "
        "cores.\n"
    );
}

void
PicklePrefetcher::processPrefetcherOutQueue()
{
    // TODO: refactor this to prefetcher scheduling class
    for (auto tracker_array: prefetcher_work_trackers) {
        for (auto tracker: tracker_array) {
            while (tracker->hasOutstandingPrefetch()) {
                PrefetchRequest prefetch_request = tracker->peekNextPrefetch();
                global_outstanding_prefetch_queue.emplace(
                    std::move(prefetch_request)
                );
                tracker->popPrefetch();
            }
        }
    }
    if (!(global_outstanding_prefetch_queue.empty())) {
        scheduleDueToOutstandingRequestsInGlobalPrefetchQueue();
    }
}

void
PicklePrefetcher::processPrefetcherInQueue()
{
    for (auto vaddr: received_packets_to_be_processed) {
        DPRINTF(
            PickleDevicePrefetcherDebug,
            "PREFETCH IN <--- vaddr 0x%llx\n", vaddr
        );
        assert(packet_status[vaddr] == PacketStatus::ARRIVED);
        for (auto tracker_array: prefetcher_work_trackers) {
            for (auto tracker: tracker_array) {
                tracker->processIncomingPrefetch(vaddr);
            }
        }
    }
    for (auto vaddr: received_packets_to_be_processed) {
        packet_data.erase(vaddr);
        packet_status.erase(vaddr);
    }
    received_packets_to_be_processed.clear();

    // we don't need to reschedule in queue event because all arrived packets
    // are processed
}

void
PicklePrefetcher::processGlobalOutstandingPrefetchQueue()
{
    while (!global_outstanding_prefetch_queue.empty()) {
        PrefetchRequest prefetch_request = \
            global_outstanding_prefetch_queue.top();
        Addr prefetchVAddr = prefetch_request.getPrefetchVAddr();
        if (packet_status.find(prefetchVAddr) != packet_status.end()) {
            DPRINTF(
                PickleDevicePrefetcherDebug,
                "PREFETCH OUT COALESCED ---> vaddr 0x%llx already in "
                "queue\n",
                prefetchVAddr
            );
            global_outstanding_prefetch_queue.pop();
            continue;
        }
        bool status = \
            owner->request_manager->enqueueLoadRequest(prefetchVAddr);
        if (status) {
            DPRINTF(
                PickleDevicePrefetcherDebug,
                "PREFETCH OUT ---> vaddr 0x%llx, priority: %lld\n",
                prefetchVAddr, prefetch_request.getPrefetchReqTime()
            );
            packet_status[prefetchVAddr] = PacketStatus::SENT;
            global_outstanding_prefetch_queue.pop();
        } else {
            DPRINTF(
                PickleDevicePrefetcherDebug, "Warn: outqueue is full\n"
            );
            break;
        }
    }
    if (!(global_outstanding_prefetch_queue.empty())) {
        scheduleDueToOutstandingRequestsInGlobalPrefetchQueue();
    }
}

void
PicklePrefetcher::replaceWorkItem()
{
    work_items.remove_if(
        [&](std::shared_ptr<WorkItem> item){
            if (item->isDone()) {
                DPRINTF(
                    PickleDevicePrefetcherWorkTrackerDebug,
                    "replaceWorkItem: remove work_id 0x%llx\n",
                    item->getWorkId()
                );
            }
            return item->isDone();
        }
    );
    while (work_items.size() < concurrent_work_item_capacity) {
        std::shared_ptr<WorkItem> work_item = \
            prefetcher_work_tracker_collective->getAndPopNextWorkItem();
        if (work_item == nullptr) {
            break;
        }
        work_items.push_back(work_item);
        populatePrefetchesFromWorkItem(work_item);
    }
    // When prefetches are populated, the prefetcher will schedule the
    // prefetch sending event.
}

void
PicklePrefetcher::populatePrefetchesFromWorkItem(
    std::shared_ptr<WorkItem> work_item
))
{
    for (auto addr: work_item->getCurrLevelExpectedPrefetches()) {
        global_outstanding_prefetch_queue.emplace(
            addr, work_item->getWorkItemReceiveTime(), work_item->getWorkId()
        );
        if (pf_vaddr_to_work_items_map.find(addr) == \
            pf_vaddr_to_work_items_map.end()) {
            pf_vaddr_to_work_items_map[addr] = \
                std::vector<std::shared_ptr<WorkItem>>();
        }
        pf_vaddr_to_work_items_map[addr].push_back(work_item);
        DPRINTF(
            PickleDevicePrefetcherDebug,
            "Adding pf_vaddr 0x%llx from WorkItem = 0x%llx\n",
            addr, work_item->getWorkId()
        );
    }
    if (!(global_outstanding_prefetch_queue.empty())) {
        scheduleDueToOutstandingRequestsInGlobalPrefetchQueue();
    }
}

uint64_t
PicklePrefetcher::getSoftwareHintPrefetchDistance() const
{
    return software_hint_prefetch_distance;
}

uint64_t
PicklePrefetcher::getPrefetchDistanceOffsetFromSoftwareHint() const
{
    return prefetch_distance_offset_from_software_hint;
}

void
PicklePrefetcher::switchOn()
{
}

void
PicklePrefetcher::switchOff()
{
}

void
PicklePrefetcher::configure(std::shared_ptr<PickleJobDescriptor> job)
{
    prefetcher_work_trackers.push_back(
        std::vector<std::shared_ptr<PrefetcherWorkTracker>>()
    );
    const uint64_t job_id = prefetcher_work_trackers.size() - 1;
    prefetcher_work_trackers.back().reserve(num_cores);
    for (int core_id = 0; core_id < num_cores; core_id++) {
        prefetcher_work_trackers.back().push_back(
            std::shared_ptr<PrefetcherWorkTracker>(
                new PrefetcherWorkTracker(this, job_id, core_id, job)
            )
        );
    }
    prefetcher_initialized = true;
}

bool
PicklePrefetcher::enqueueWork(
    const uint64_t workData, const uint64_t prefetchKernelId,
    const uint64_t cpuId
)
{
    if (!prefetcher_initialized)
        return false;
    workCount++;
    prefetcherStats.numReceivedWork++;
    if (workCount % 1000 == 0 || workCount == 1) {
        DPRINTF(
            PickleDevicePrefetcherProgressTracker,
            "Work sent: %lld, numPrefetches: %lld, inQ size: %lld\n",
            workCount, prefetcherStats.numPrefetches.value(),
            packet_status.size()
        );
    }
    // For BFS: workData = curr_ptr + sw_prefetch_distance * 4 of the workQueue
    // For PR: workData = node_id + sw_prefetch_distance
    prefetcher_work_trackers[prefetchKernelId][cpuId]->addWorkItem(workData);
    scheduleDueToNewOutstandingPrefetchRequests();
    DPRINTF(
        PickleDevicePrefetcherDebug,
        "NEW WORK: data = 0x%llx\n", workData
    );
    return true;
}

void
PicklePrefetcher::receivePrefetch(
  const uint64_t vaddr, std::unique_ptr<uint8_t[]> p
)
{
    prefetcherStats.numPrefetches++;
    DPRINTF(
        PickleDevicePrefetcherDebug,
        "Receiving Packet: vaddr = 0x%llx\n", vaddr
    );
    // Update the packet status
    packet_status[vaddr] = PacketStatus::ARRIVED;
    packet_data[vaddr] = std::move(p);
    received_packets_to_be_processed.insert(vaddr);
    // Trigger In Queue Processing
    scheduleDueToIncomingPrefetch();
}

void
PicklePrefetcher::scheduleDueToIncomingPrefetch()
{
    if (!processInQueueEvent.scheduled()) {
        schedule(
            processInQueueEvent, curTick() + ticks_per_cycle
        );
    }
}

void
PicklePrefetcher::scheduleDueToNewOutstandingPrefetchRequests()
{
    if (!processOutQueueEvent.scheduled()) {
        schedule(
            processOutQueueEvent, curTick() + ticks_per_cycle
        );
    }
}

void
PicklePrefetcher::scheduleDueToOutstandingRequestsInGlobalPrefetchQueue()
{
    if (!processGlobalOutstandingPrefetchQueueEvent.scheduled()) {
        schedule(
            processGlobalOutstandingPrefetchQueueEvent,
            curTick() + ticks_per_cycle
        );
    }
}

void
PicklePrefetcher::scheduleWorkItemReplacement()
{
    if (!replaceWorkItemsEvent.scheduled()) {
        schedule(
            replaceWorkItemsEvent, curTick() + ticks_per_cycle
        );
    }
}

PacketPtr
PicklePrefetcher::zeroCycleLoadWithVAddr(const Addr& vaddr, bool& success)
{
    return owner->zeroCycleLoadWithVAddr(vaddr, success);
}
PacketPtr
PicklePrefetcher::zeroCycleLoadWithPAddr(const Addr& paddr, bool& success)
{
    return owner->zeroCycleLoadWithPAddr(paddr, success);
}

void
PicklePrefetcher::profilePrefetchWithUnknownVAddr()
{
    prefetcherStats.numUnknownPrefetches++;
}

void
PicklePrefetcher::regStats()
{
    ClockedObject::regStats();
    prefetcherStats.regStats();
}

PicklePrefetcher::PrefetcherStats::PrefetcherStats(
    statistics::Group *parent
) : statistics::Group(parent),
    ADD_STAT(
        numReceivedWork, statistics::units::Count::get(),
        "Number of work sent from software"
    ),
    ADD_STAT(
        numPrefetches, statistics::units::Count::get(),
        "Number of prefetch requests generated"
    ),
    ADD_STAT(
        numUnknownPrefetches, statistics::units::Count::get(),
        "Number of prefetches received that does not fit into any array"
    ),
    ADD_STAT(
        histInQueueLength, statistics::units::Count::get(),
        "Histogram of the in queue length over time"
    ),
    ADD_STAT(
        histOutQueueLength, statistics::units::Count::get(),
        "Histogram of the out queue length over time"
    )
{
    histInQueueLength
      .init(16)
      .flags(statistics::pdf);
    histOutQueueLength
      .init(16)
      .flags(statistics::pdf);
}

void
PicklePrefetcher::PrefetcherStats::regStats()
{
}

PicklePrefetcher::TaskStats::TaskStats(
    statistics::Group *parent,
    const uint64_t jobId, const uint64_t cpuId
) : statistics::Group(
        parent, csprintf("cpu_%d.job_%d", cpuId, jobId).c_str()
    ),
    ADD_STAT(
        taskCount, statistics::units::Count::get(),
        csprintf("Number of tasks from core %d", cpuId).c_str()
    ),
    //ADD_STAT(
    //    queuedTime, statistics::units::Tick::get(),
    //    "From task receive to first prefetch"
    //),
    ADD_STAT(
        prefetchLv0Time, statistics::units::Tick::get(),
        "From first prefetch to final prefetch of array 0"
    ),
    ADD_STAT(
        prefetchLv1Time, statistics::units::Tick::get(),
        "From first prefetch to final prefetch of array 1"
    ),
    ADD_STAT(
        prefetchLv2Time, statistics::units::Tick::get(),
        "From first prefetch to final prefetch of array 2"
    ),
    ADD_STAT(
        prefetchLv3Time, statistics::units::Tick::get(),
        "From first prefetch to final prefetch of array 3"
    ),
    ADD_STAT(
        totalPrefetchTime, statistics::units::Tick::get(),
        "From first prefetch to final prefetch"
    ),
    ADD_STAT(
        timelyPrefetchesDistance,
        statistics::units::Tick::get(),
        "Time from prefetch complete to core consumption for timely prefetches"
    ),
    ADD_STAT(
        latePrefetchesDistance,
        statistics::units::Tick::get(),
        "Time from core consumption to prefetch complete for late prefetches"
    )
{
    //queuedTime
    //  .init(16)
    //  .flags(statistics::pdf);
    prefetchLv0Time
      .init(16)
      .flags(statistics::pdf);
    prefetchLv1Time
      .init(16)
      .flags(statistics::pdf);
    prefetchLv2Time
      .init(16)
      .flags(statistics::pdf);
    prefetchLv3Time
      .init(16)
      .flags(statistics::pdf);
    totalPrefetchTime
      .init(16)
      .flags(statistics::pdf);
    timelyPrefetchesDistance
      .init(16)
      .flags(statistics::pdf);
    latePrefetchesDistance
      .init(16)
      .flags(statistics::pdf);
}

void
PicklePrefetcher::TaskStats::regStats()
{
}

void
PicklePrefetcher::profileWork(
    std::shared_ptr<WorkItem> work,
    const uint64_t job_id, const uint64_t core_id
)
{
    DPRINTF(
        PickleDevicePrefetcherWorkTrackerDebug,
        "profileWork: job_id: %lld, core_id: %lld, work_id 0x%llx\n",
        job_id, core_id, work->getWorkId()
    );
    std::shared_ptr<TaskStats> task_stat = taskStats[job_id][core_id];
    task_stat->taskCount++;;
    task_stat->prefetchLv0Time.sample(work->getPrefetchLvTime(0));
    task_stat->prefetchLv1Time.sample(work->getPrefetchLvTime(1));
    task_stat->prefetchLv2Time.sample(work->getPrefetchLvTime(2));
    task_stat->prefetchLv3Time.sample(work->getPrefetchLvTime(3));
    task_stat->totalPrefetchTime.sample(work->getTotalPrefetchTime());
    if (work->hasCoreWorkedOnThisWork()) { // late prefetch
        DPRINTF(
            PickleDevicePrefetcherWorkTrackerDebug,
            "profileWork: late pf, complete time: %lld, core use: %lld\n",
            work->getPrefetchCompleteTime(), work->getCoreUseTime()
        );
        task_stat->latePrefetchesDistance
            .sample(
                work->getPrefetchCompleteTime() - work->getCoreUseTime()
            );
    } else { // timely prefetch
        // nothing to do here as the core has not worked on this work item
        DPRINTF(
            PickleDevicePrefetcherWorkTrackerDebug,
            "profileWork: maybe timely pf\n"
        );
    }
}

void
PicklePrefetcher::profileTimelyPrefetch(
    const Tick pf_complete_time,
    const uint64_t job_id, const uint64_t core_id
)
{
    taskStats[job_id][core_id]->timelyPrefetchesDistance
        .sample(
            curTick() - pf_complete_time
        );
}

}; // namespace gem5
