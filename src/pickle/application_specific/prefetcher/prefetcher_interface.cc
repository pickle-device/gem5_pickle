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

#include "pickle/application_specific/prefetcher/prefetcher_interface.hh"

#include "debug/PickleDevicePrefetcherDebug.hh"
#include "debug/PickleDevicePrefetcherProgressTracker.hh"
#include "debug/PickleDevicePrefetcherTrace.hh"

#include "pickle/device/pickle_device.hh"
#include "pickle/request_manager/manager.hh"

namespace gem5
{

PrefetcherInterface::PrefetcherInterface(
  const PrefetcherInterfaceParams &params
)
  : ClockedObject(params),
    prefetch_distance(params.prefetch_distance),
    prefetch_distance_offset_from_software_hint(
      params.prefetch_distance_offset_from_software_hint
    ),
    processInQueueEvent(
        [this]{processPrefetcherInQueue();},
        name() + ".operate_prefetcher_in_queue_event"
    ),
    processOutQueueEvent(
        [this]{processPrefetcherOutQueue();},
        name() + ".operate_prefetcher_out_queue_event"
    ),
    ticks_per_cycle(1000),
    prefetcher_initialized(false),
    owner(nullptr),
    workCount(0),
    prefetcherStats(this)
{
    panic_if(
        prefetch_distance < prefetch_distance_offset_from_software_hint,
        "Prefetch distance offset from software hint must not be greater "
        "than the prefetch distance\n"
    );
}

PrefetcherInterface::~PrefetcherInterface()
{
}

void
PrefetcherInterface::startup()
{
}

void
PrefetcherInterface::setOwner(PickleDevice* pickle_device)
{
    this->owner = pickle_device;
    this->ticks_per_cycle = pickle_device->getNumTicksPerCycle();
    this->num_cores = owner->getNumCores();
    for (int i = 0; i < num_cores; ++i) {
        prefetcher_work_trackers.push_back(
            std::shared_ptr<PrefetcherWorkTracker>(
                new PrefetcherWorkTracker(this, i)
            )
        );
    }
}

void
PrefetcherInterface::processPrefetcherOutQueue()
{
    // TODO: a better scheduling policy?
    for (auto tracker: prefetcher_work_trackers) {
        while (tracker->hasOutstandingPrefetch()) {
            Addr prefetchVAddr = tracker->peekNextPrefetch();
            bool status = \
                owner->request_manager->enqueueLoadRequest(prefetchVAddr);
            if (status) {
                DPRINTF(
                    PickleDevicePrefetcherDebug,
                    "PREFETCH OUT ---> vaddr 0x%llx\n",
                    prefetchVAddr
                );
                packet_status[prefetchVAddr] = PacketStatus::SENT;
                tracker->popPrefetch();
            } else {
                DPRINTF(
                    PickleDevicePrefetcherDebug, "Warn: outqueue is full\n"
                );
                scheduleDueToNewOutstandingPrefetchRequests();
                break;
            }
        }
    }
}

void
PrefetcherInterface::processPrefetcherInQueue()
{
    std::vector<Addr> to_be_removed;
    to_be_removed.reserve(16);
    for (auto& [vaddr, status] : packet_status)
    {
        const bool is_ready = (status == PacketStatus::ARRIVED);
        if (!is_ready)
            continue;
        DPRINTF(
            PickleDevicePrefetcherDebug,
            "PREFETCH IN <--- vaddr 0x%llx\n", vaddr
        );
        for (auto tracker : prefetcher_work_trackers)
        {
            tracker->processIncomingPrefetch(vaddr);
        }
        to_be_removed.push_back(vaddr);
    }
    for (auto const& vaddr: to_be_removed)
    {
        packet_data.erase(vaddr);
        packet_status.erase(vaddr);
    }

    // we don't need to reschedule in queue event because all arrived packets
    // are processed
}

uint64_t
PrefetcherInterface::getPrefetchDistance() const
{
    return prefetch_distance;
}

uint64_t
PrefetcherInterface::getPrefetchDistanceOffsetFromSoftwareHint() const
{
    return prefetch_distance_offset_from_software_hint;
}

void
PrefetcherInterface::switchOn()
{
}

void
PrefetcherInterface::switchOff()
{
}

void
PrefetcherInterface::configure(std::shared_ptr<PickleJobDescriptor> job)
{
    for (auto tracker: prefetcher_work_trackers) {
        tracker->setJobDescriptor(job);
    }
    prefetcher_initialized = true;
}

bool
PrefetcherInterface::enqueueWork(
    const uint64_t workData, const uint64_t cpuId
)
{
    if (!prefetcher_initialized)
        return false;
    workCount++;
    Addr prefetchAddr = \
        workData - prefetch_distance_offset_from_software_hint * 4;
    prefetcherStats.numReceivedWork++;
    if (workCount % 1000 == 0 || workCount == 1) {
        DPRINTF(
            PickleDevicePrefetcherProgressTracker,
            "Work sent: %lld, numPrefetches: %lld, inQ size: %lld\n",
            workCount, prefetcherStats.numPrefetches.value(),
            packet_status.size()
        );
    }
    prefetcher_work_trackers[cpuId]->addWorkItem(prefetchAddr);
    scheduleDueToNewOutstandingPrefetchRequests();
    DPRINTF(
        PickleDevicePrefetcherDebug,
        "NEW WORK: data = 0x%llx\n", prefetchAddr
    );
    return true;
}

void
PrefetcherInterface::receivePrefetch(
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
    // Trigger In Queue Processing
    scheduleDueToIncomingPrefetch();
}

void
PrefetcherInterface::scheduleDueToIncomingPrefetch()
{
    if (!processInQueueEvent.scheduled()) {
        schedule(
            processInQueueEvent, curTick() + ticks_per_cycle
        );
    }
}

void
PrefetcherInterface::scheduleDueToNewOutstandingPrefetchRequests()
{
    if (!processOutQueueEvent.scheduled()) {
        schedule(
            processOutQueueEvent, curTick() + ticks_per_cycle
        );
    }
}

PacketPtr
PrefetcherInterface::zeroCycleLoadWithVAddr(const Addr& vaddr, bool& success)
{
    return owner->zeroCycleLoadWithVAddr(vaddr, success);
}
PacketPtr
PrefetcherInterface::zeroCycleLoadWithPAddr(const Addr& paddr, bool& success)
{
    return owner->zeroCycleLoadWithPAddr(paddr, success);
}

void
PrefetcherInterface::profilePrefetchWithUnknownVAddr()
{
    prefetcherStats.numUnknownPrefetches++;
}

void
PrefetcherInterface::regStats()
{
    ClockedObject::regStats();
    prefetcherStats.regStats();
}

PrefetcherInterface::PrefetcherStats::PrefetcherStats(
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
PrefetcherInterface::PrefetcherStats::regStats()
{
}

PrefetcherInterface::TaskStats::TaskStats(
    statistics::Group *parent, const uint64_t cpuId
) : statistics::Group(parent, csprintf("cpu_%d", cpuId).c_str()),
    ADD_STAT(
        taskCount, statistics::units::Count::get(),
        csprintf("Number of tasks from core %d", cpuId).c_str()
    ),
    ADD_STAT(
        queuedTime, statistics::units::Tick::get(),
        "From task receive to first prefetch"
    ),
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
        prefetchCompleteToCoreConsumptionTimeForTimelyPrefetches,
        statistics::units::Tick::get(),
        "Time from prefetch complete to core consumption for timely prefetches"
    ),
    ADD_STAT(
        coreConsumptionToPrefetchCompleteTimeForLatePrefetches,
        statistics::units::Tick::get(),
        "Time from core consumption to prefetch complete for late prefetches"
    )
{
    queuedTime
      .init(16)
      .flags(statistics::pdf);
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
    prefetchCompleteToCoreConsumptionTimeForTimelyPrefetches
      .init(16)
      .flags(statistics::pdf);
    coreConsumptionToPrefetchCompleteTimeForLatePrefetches
      .init(16)
      .flags(statistics::pdf);
}

void
PrefetcherInterface::TaskStats::regStats()
{
}

}; // namespace gem5
