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
#include "pickle/device/pickle_device.hh"

namespace gem5
{

PrefetcherInterface::PrefetcherInterface(
  const PrefetcherInterfaceParams &params
)
  : ClockedObject(params),
    prefetcher_initialized(false),
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
PrefetcherInterface::processPrefetcherOutQueue()
{
}

void
PrefetcherInterface::processPrefetcherInQueue()
{
}

void
PrefetcherInterface::setOwner(PickleDevice* pickle_device)
{
    this->owner = pickle_device;
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
PrefetcherInterface::clockTick()
{
}

void
PrefetcherInterface::configure(const PickleJobDescriptor& job)
{
}

bool
PrefetcherInterface::enqueueWork(const uint64_t& node_id)
{
    return false; // TODO: implement
}

void
PrefetcherInterface::receivePrefetch(
  const uint64_t& vaddr, std::unique_ptr<uint8_t[]> p
)
{
}

void
PrefetcherInterface::regStats()
{
}

PrefetcherInterface::PrefetcherStats::PrefetcherStats(
    statistics::Group *parent
) : statistics::Group(parent),
    ADD_STAT(
        activatedTick, statistics::units::Tick::get(),
        "The tick where the prefetcher was activate"
    ),
    ADD_STAT(
        activatedCycle, statistics::units::Tick::get(),
        "The cycle where the prefetcher was activate"
    ),
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
