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
    prefetch_dropping_distance(
        params.prefetch_dropping_distance
    ),
    processInQueueEvent(
        [this]{processPrefetcherInQueue();},
        name() + ".operate_prefetcher_in_queue_event"
    ),
    processOutgoingPrefetchRequestQueueEvent(
        [this]{processOutgoingPrefetchRequestQueue();},
        name() + ".process_outstanding_prefetch_requests_event"
    ),
    ticks_per_cycle(1000),
    num_cores(params.num_cores),
    prefetcher_initialized(false),
    num_received_jobs(0),
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

    prefetcher_work_tracker_collective =
        std::shared_ptr<PrefetcherWorkTrackerCollective>(
            new PrefetcherWorkTrackerCollective(concurrent_work_item_capacity)
        );
    prefetcher_work_tracker_collective->setOwner(this);

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
PicklePrefetcher::processPrefetcherInQueue()
{
    for (auto vaddr: received_packets_to_be_processed) {
        DPRINTF(
            PickleDevicePrefetcherDebug,
            "PREFETCH IN <--- vaddr 0x%llx\n", vaddr
        );
        assert(packet_status[vaddr] == PacketStatus::ARRIVED);
        prefetcher_work_tracker_collective->processIncomingPrefetch(vaddr);
    }
    for (auto vaddr: received_packets_to_be_processed) {
        packet_data.erase(vaddr);
        packet_status.erase(vaddr);
    }
    received_packets_to_be_processed.clear();
}

void
PicklePrefetcher::processOutgoingPrefetchRequestQueue()
{
    while (prefetcher_work_tracker_collective->hasOutstandingPrefetchRequest())
    {
        PrefetchRequest prefetch_request = \
            prefetcher_work_tracker_collective->peekNextPrefetchRequest();
        Addr prefetchVAddr = prefetch_request.getPrefetchVAddr();
        if (packet_status.find(prefetchVAddr) != packet_status.end()) {
            DPRINTF(
                PickleDevicePrefetcherDebug,
                "PREFETCH OUT COALESCED ---> vaddr 0x%llx already in "
                "queue\n",
                prefetchVAddr
            );
            prefetcher_work_tracker_collective->popPrefetchRequest();
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
            prefetcher_work_tracker_collective->popPrefetchRequest();
        } else {
            DPRINTF(
                PickleDevicePrefetcherDebug, "Warn: outqueue is full\n"
            );
            break;
        }
    }
    if (prefetcher_work_tracker_collective->hasOutstandingPrefetchRequest()) {
        scheduleDueToOutstandingPrefetchRequests();
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
    num_received_jobs++;
    const uint64_t job_id = num_received_jobs - 1;
    for (int core_id = 0; core_id < num_cores; core_id++) {
        prefetcher_work_tracker_collective->addPrefetcherWorkTracker(
            job_id, core_id,
            std::shared_ptr<PrefetcherWorkTracker>(
                new PrefetcherWorkTracker(
                    this, prefetcher_work_tracker_collective,
                    job_id, core_id, job, prefetch_dropping_distance
                )
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
    prefetcher_work_tracker_collective->getPrefetcherWorkTracker(
        prefetchKernelId, cpuId
    )->addWorkItem(workData);
    // Check if there is an empty slot for the new work item
    prefetcher_work_tracker_collective->replaceActiveWorkItemsUponCompletion();
    //scheduleDueToOutstandingPrefetchRequests();
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
PicklePrefetcher::scheduleDueToOutstandingPrefetchRequests()
{
    if (!processOutgoingPrefetchRequestQueueEvent.scheduled()) {
        schedule(
            processOutgoingPrefetchRequestQueueEvent,
            curTick() + ticks_per_cycle
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
        numActivatedWork, statistics::units::Count::get(),
        "Number of work activated by the prefetcher"
    ),
    ADD_STAT(
        numWorkedDroppedDueToCoreFinished, statistics::units::Count::get(),
        "Number of work items dropped because the core has already worked on "
        "them"
    ),
    ADD_STAT(
        numWorkedDroppedDueToCoreTooClose, statistics::units::Count::get(),
        "Number of work items dropped because the core is too close to the "
        "prefetch distance even though the core has not worked on them yet"
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
        "From the first prefetch to final prefetch"
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
    ),
    ADD_STAT(
        coreStartedBeforePrefetchStarted,
        statistics::units::Tick::get(),
        "Time from samples that the core started before the prefetch started"
    ),
    ADD_STAT(
        coreStartedBeforePrefetchComplete,
        statistics::units::Tick::get(),
        "Time from samples that the core started after the prefetch started"
        "and before the prefetch complete"
    ),
    ADD_STAT(
        prefetchCompleteBeforeCoreStarted,
        statistics::units::Tick::get(),
        "Time from samples that the prefetch complete before the core started"
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
    coreStartedBeforePrefetchStarted
      .init(16)
      .flags(statistics::pdf);
    coreStartedBeforePrefetchComplete
      .init(16)
      .flags(statistics::pdf);
    prefetchCompleteBeforeCoreStarted
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
    uint64_t work_id = work->getWorkId();
    task_stat->taskCount++;;
    task_stat->prefetchLv0Time.sample(work->getPrefetchLvTime(0));
    task_stat->prefetchLv1Time.sample(work->getPrefetchLvTime(1));
    task_stat->prefetchLv2Time.sample(work->getPrefetchLvTime(2));
    task_stat->prefetchLv3Time.sample(work->getPrefetchLvTime(3));
    task_stat->totalPrefetchTime.sample(work->getTotalPrefetchTime());
    // late prefetch?
    if (prefetcher_work_tracker_collective->hasCoreWorkedOnThisWork(
        job_id, work->getWorkId()
    )) {
        const Tick core_use_time = prefetcher_work_tracker_collective
                ->getCoreStartTime(job_id, work_id);
        DPRINTF(
            PickleDevicePrefetcherWorkTrackerDebug,
            "profileWork: late pf, complete time: %lld, core use: %lld\n",
            work->getPrefetchCompleteTime(), core_use_time
        );
        task_stat->latePrefetchesDistance.sample(
                work->getPrefetchCompleteTime() - core_use_time
            );
        if (core_use_time < work->getWorkActivationTime()) {
            task_stat->coreStartedBeforePrefetchStarted.sample(
                work->getWorkActivationTime() - core_use_time
            );
        } else if (core_use_time < work->getPrefetchCompleteTime()) {
            task_stat->coreStartedBeforePrefetchComplete.sample(
                work->getPrefetchCompleteTime() - core_use_time
            );
        }
        prefetcher_work_tracker_collective->untrackCoreStartTime(
            job_id, work_id
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
    const Tick pf_complete_time, const Tick core_start_time,
    const uint64_t job_id, const uint64_t core_id
)
{
    taskStats[job_id][core_id]->timelyPrefetchesDistance.sample(
        core_start_time - pf_complete_time
    );
    taskStats[job_id][core_id]->prefetchCompleteBeforeCoreStarted.sample(
        core_start_time - pf_complete_time
    );
}

}; // namespace gem5
