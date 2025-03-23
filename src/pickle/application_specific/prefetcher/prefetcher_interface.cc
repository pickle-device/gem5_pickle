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

#include "pickle/application_specific/prefetcher/backend/cerebellum.hpp"
#include "pickle/device/pickle_device.hh"
#include "pickle/request_manager/manager.hh"

namespace gem5
{

PrefetcherInterface::PrefetcherInterface(
  const PrefetcherInterfaceParams &params
)
  : ClockedObject(params),
    prefetch_distance(params.prefetch_distance),
    prefetcher(nullptr),
    prefetcher_initialized(false),
    owner(nullptr),
    workCount(0),
    prefetcherStats(this)
{
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
}

void
PrefetcherInterface::clockTick()
{
    if (!prefetcher_initialized)
        return;
    prefetcher->operate(curTick());
    prefetcherStats.histInQueueLength.sample(packet_status.size());
    prefetcherStats.histOutQueueLength.sample(prefetcher->outQSize());
    processPrefetcherOutQueue();
    processPrefetcherInQueue();
}

void
PrefetcherInterface::processPrefetcherOutQueue()
{
    if (prefetcher->outQSize() > 0)
    {
        DPRINTF(
            PickleDevicePrefetcherDebug,
            "OutQueue size: %d\n", prefetcher->outQSize()
        );
    }
    while (prefetcher->hasNextPf()) // TODO: do we want to limit how many
                                    // requests we send out per cycle?
    {
        Addr nextPfVAddr = prefetcher->nextPf();
        bool status = owner->request_manager->enqueueLoadRequest(nextPfVAddr);
        if (status) {
            DPRINTF(
                PickleDevicePrefetcherDebug,
                "PREFETCH OUT ---> vaddr 0x%llx\n",
                nextPfVAddr
            );
            packet_status[nextPfVAddr] = PacketStatus::SENT;
            prefetcher->popNextPf(curTick());
        } else {
            DPRINTF(
                PickleDevicePrefetcherDebug, "Warn: outqueue is full\n"
            );
            break;
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
        std::vector<uint64_t> triggerAddrs = \
            prefetcher->captureResponse(
                curTick(), vaddr, std::move(packet_data[vaddr]),
                64 //block_size
            );
        to_be_removed.push_back(vaddr);
    }

    for (auto const& vaddr: to_be_removed)
    {
        packet_data.erase(vaddr);
        packet_status.erase(vaddr);
    }
}

uint64_t
PrefetcherInterface::getPrefetchDistance() const
{
    return prefetch_distance;
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
PrefetcherInterface::configure(const PickleJobDescriptor& job)
{
    prefetcher = std::unique_ptr<c_cerebellum>(new c_cerebellum(this));
    // converting to the backend format
    std::vector<std::tuple<
        uint64_t, uint64_t, bool, bool, uint64_t, uint64_t, uint64_t
    >> jobTuples;
    for (const auto& array : job.arrays)
    {
        jobTuples.push_back(
            std::make_tuple(
                array.array_id,
                array.dst_id,
                array.is_indexed_access,
                array.is_ranged_access,
                array.vaddr_start,
                array.vaddr_end,
                array.element_size
            )
        );
        DPRINTF(
            PickleDevicePrefetcherDebug,
            "Prefetcher configured: array_id %lld, dst_id %lld, is_indexed %d"
            ", is_ranged %d, vaddr_start 0x%llx, vaddr_end 0x%llx"
            " element_size %lld\n",
            array.array_id, array.dst_id,
            array.is_indexed_access, array.is_ranged_access,
            array.vaddr_start, array.vaddr_end, array.element_size
        );
    }
    prefetcher->configure(jobTuples);
    prefetcher_initialized = true;
}

bool
PrefetcherInterface::enqueueWork(const uint64_t& workData)
{
    if (!prefetcher_initialized)
        return false;
    workCount++;
    prefetcherStats.numReceivedWork++;
    if (workCount % 1000 == 0 || workCount == 1) {
        DPRINTF(
            PickleDevicePrefetcherProgressTracker,
            "Work sent: %d\n",
            workCount
        );
    }
    prefetcher->captureRequest(curTick(), workData);
    DPRINTF(
        PickleDevicePrefetcherDebug,
        "NEW WORK: data = 0x%llx\n", workData
    );
    return true;
}

void
PrefetcherInterface::receivePrefetch(
  const uint64_t& vaddr, std::unique_ptr<uint8_t[]> p
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
        "Number of prefetches received"
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

}; // namespace gem5
