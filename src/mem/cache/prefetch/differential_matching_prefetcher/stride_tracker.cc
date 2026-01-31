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

#include "mem/cache/prefetch/differential_matching_prefetcher/stride_tracker.hh"

namespace gem5
{
namespace prefetch
{

StrideTrackerEntry::StrideTrackerEntry(
  const Addr _pc, const Addr _block_address, const Tick _access_timestamp,
  const double _confidence_threshold
) : pc(_pc),
    access_timestamp(_access_timestamp),
    previous_stride(0),
    previous_effective_address(_block_address),
    confidence(4),
    confidence_threshold(_confidence_threshold)
{
}

void
StrideTrackerEntry::update(
  const Addr block_address, const Tick access_timestamp
)
{
    int64_t current_stride =
      static_cast<int64_t>(block_address) -
      static_cast<int64_t>(previous_effective_address);

    // Update the timestamp and previous effective address
    this->access_timestamp = access_timestamp;
    this->previous_effective_address = block_address;

    // We don't update on same block accesses
    if (current_stride == 0) {
        // No stride change for same address access
        return;
    }

    if (current_stride == previous_stride) {
        confidence++;
    } else {
        confidence--;
        if (confidence.calcSaturation() < confidence_threshold) {
            previous_stride = current_stride;
        }
    }

    DMP_STRIDE_TRACKER_DEBUG(
        "PC %#x updated: previous_stride=%ld, "
        "current_stride=%ld, confidence=%f, threshold=%f\n",
        pc, previous_stride, current_stride, confidence.calcSaturation(),
        confidence_threshold
    );
}

bool StrideTrackerEntry::isConfident() const
{
    return confidence.calcSaturation() >= confidence_threshold;
}

StrideTracker::StrideTracker(
  const uint64_t _capacity, const double _confidence_threshold,
  const uint64_t _cache_block_size,
  DifferentialMatchingPrefetcherInterface *_prefetcher_interface
) : capacity(_capacity),
    confidence_threshold(_confidence_threshold),
    cache_block_size(_cache_block_size),
    prefetcher_interface(_prefetcher_interface)
{
    stride_tracker.reserve(capacity);
}

void
StrideTracker::replaceLeastRecentlyUsedEntry(
    const Addr pc, const Addr block_address, const Tick access_timestamp
)
{
    if (stride_tracker.size() < capacity) {
        stride_tracker.emplace_back(
            pc, block_address, access_timestamp, confidence_threshold
        );
        DMP_STRIDE_TRACKER_DEBUG(
            "Added new entry for PC %#x\n", pc
        );
        return;
    }

    // Find the LRU entry
    auto lru_it = stride_tracker.begin();
    for (auto it = stride_tracker.begin(); it != stride_tracker.end(); ++it) {
        if (it->access_timestamp < lru_it->access_timestamp) {
            lru_it = it;
        }
    }
    // Replace the LRU entry with the new one
    *lru_it = StrideTrackerEntry(pc, block_address, access_timestamp,
                                confidence_threshold);
    DMP_STRIDE_TRACKER_DEBUG(
        "Replaced LRU entry with new entry for PC %#x\n", pc
    );
}

void
StrideTracker::track(
  const Addr pc, const Addr block_address, const Tick access_timestamp
)
{
    // Check if the PC already exists in the stride tracker
    for (auto it = stride_tracker.begin(); it != stride_tracker.end(); ++it) {
        if (it->pc == pc) {
            // Found the entry, now we check confidence before updating
            const bool was_confident = it->isConfident();
            // Update the existing entry
            it->update(block_address, access_timestamp);
            // If we just recently became confident, notify the prefetcher
            if (!was_confident && it->isConfident()) {
                prefetcher_interface->handleNewlyDetectedStride(pc);
                DMP_STRIDE_TRACKER_DEBUG(
                    "PC %#x became confident with stride %ld\n",
                    pc, it->previous_stride
                );
            }
            return;
        }
    }

    // If not found, add a new entry (possibly replacing an old one)
    replaceLeastRecentlyUsedEntry(pc, block_address, access_timestamp);
}

} // namespace prefetch
} // namespace gem5
