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

#ifndef __DMP_STRIDE_TRACKER_HH__
#define __DMP_STRIDE_TRACKER_HH__

#include <cstdint>
#include <optional>
#include <vector>

#include "base/logging.hh"
#include "base/sat_counter.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "debug/DifferentialMatchingPrefetcherStrideTrackerDebug.hh"
#include "mem/cache/prefetch/differential_matching_prefetcher/differential_matching_prefetcher_interface.hh"

#define DMP_STRIDE_TRACKER_DEBUG(...) \
    DPRINTF(DifferentialMatchingPrefetcherStrideTrackerDebug, \
            "(Stride Tracker) " __VA_ARGS__)

namespace gem5
{

namespace prefetch
{

namespace dmp
{

class StrideTrackerEntry
{
  public:
    Addr pc;
    Tick access_timestamp;
    int64_t previous_stride;
    Addr previous_effective_address;
    SatCounter8 confidence;
    double confidence_threshold;
  public:
    StrideTrackerEntry(
      const Addr _pc, const Addr _block_address, const Tick _access_timestamp,
      const double _confidence_threshold
    );
    void update(
      const Addr block_address, const Tick access_timestamp
    );
    bool isConfident() const;
};

class StrideTracker
{
  private:
    const uint64_t capacity;
    const double confidence_threshold;
    const uint64_t cache_block_size;
    std::vector<StrideTrackerEntry> stride_tracker;
    DifferentialMatchingPrefetcherInterface *prefetcher_interface;
    void replaceLeastRecentlyUsedEntry(
        const Addr pc, const Addr block_address, const Tick access_timestamp
    );
  public:
    StrideTracker(
      const uint64_t _capacity, const double _confidence_threshold,
      const uint64_t _cache_block_size,
      DifferentialMatchingPrefetcherInterface *_prefetcher_interface
    );
    void track(
      const Addr pc, const Addr block_address, const Tick access_timestamp
    );
};

} // namespace dmp

} // namespace prefetch

} // namespace gem5

#endif // __DMP_STRIDE_TRACKER_HH__
