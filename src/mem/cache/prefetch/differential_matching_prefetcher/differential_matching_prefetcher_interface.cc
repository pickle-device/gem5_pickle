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

#include "mem/cache/prefetch/differential_matching_prefetcher/differential_matching_prefetcher_interface.hh"

#include <cstdint>

#include "base/statistics.hh"
#include "base/stats/group.hh"

namespace gem5
{

namespace prefetch
{

namespace dmp
{

PrefetcherStats::PrefetcherStats(statistics::Group *parent)
  : statistics::Group(parent, "PrefetcherStats"),
    ADD_STAT(
        numPrefetchableL1CacheHits, statistics::units::Count::get(),
        "Number of observed L1 cache hits"
    ),
    ADD_STAT(
        numPrefetchableL1CacheMisses, statistics::units::Count::get(),
        "Number of observed L1 cache misses"
    ),
    ADD_STAT(
        numStridePrefetchesSentToIRT, statistics::units::Count::get(),
        "Number of stride prefetches sent to IRT"
    ),
    ADD_STAT(
        numStridePrefetchesEmitted, statistics::units::Count::get(),
        "Number of stride prefetches emitted"
    ),
    ADD_STAT(
        numStridePrefetchesDroppedDueToOutOfMemoryBounds,
        statistics::units::Count::get(),
        "Number of stride prefetches dropped due to out of memory bounds"
    ),
    ADD_STAT(
        numStridePrefetchesDroppedDueToCrossBlockAccesses,
        statistics::units::Count::get(),
        "Number of stride prefetches dropped due to cross-block accesses"
    ),
    ADD_STAT(
        numDMPPrefetchesEmitted, statistics::units::Count::get(),
        "Number of DMP prefetches emitted"
    ),
    ADD_STAT(
        numDMPPrefetchesDroppedDueToOutOfMemoryBounds,
        statistics::units::Count::get(),
        "Number of DMP prefetches dropped due to out of memory bounds"
    ),
    ADD_STAT(
        numDMPPrefetchesDroppedDueToCrossBlockAccesses,
        statistics::units::Count::get(),
        "Number of DMP prefetches dropped due to cross-block accesses"
    )
{
}

} // namespace dmp

} // namespace prefetch

} // namespace gem5
