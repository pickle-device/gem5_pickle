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

#include "base/types.hh"
#include "mem/cache/prefetch/differential_matching_prefetcher/util.hh"

namespace gem5
{

namespace prefetch
{

namespace dmp
{

StrideTrackerEntry::StrideTrackerEntry(
  const Addr _pc, const uint64_t _access_size, const Addr _block_address,
  const Tick _access_timestamp, const double _confidence_threshold,
  DifferentialMatchingPrefetcherInterface *prefetcher_interface
) : pc(_pc),
    access_size(_access_size),
    access_timestamp(_access_timestamp),
    previous_stride(0),
    previous_effective_address(_block_address),
    confidence(/*bits*/3, /*initial_value*/2),
    confidence_threshold(_confidence_threshold),
    prefetcher_interface(prefetcher_interface)
{
}

void
StrideTrackerEntry::update(
  const Addr block_address, const Tick access_timestamp
)
{
    const int64_t current_stride =
      static_cast<int64_t>(block_address) -
      static_cast<int64_t>(previous_effective_address);
    const int64_t previous_stride_for_debugging = this->previous_stride;

    // Update the timestamp and previous effective address
    this->access_timestamp = access_timestamp;
    this->previous_effective_address = block_address;

    // We don't update on same block accesses
    if (current_stride == 0) {
        // No stride change for same address access
        return;
    }

    if (current_stride == this->previous_stride) {
        confidence++;
    } else {
        confidence--;
        if (confidence.calcSaturation() < confidence_threshold) {
            this->previous_stride = current_stride;
        }
    }

    DMP_STRIDE_TRACKER_DEBUG(
        "PC %#x updated: previous_stride=%ld, "
        "current_stride=%ld, confidence=%f, threshold=%f\n",
        pc, previous_stride_for_debugging, current_stride,
        confidence.calcSaturation(), confidence_threshold
    );
}

bool StrideTrackerEntry::isConfident() const
{
    return confidence.calcSaturation() >= confidence_threshold;
}

StrideTracker::StrideTracker(
  const uint64_t _capacity, const double _confidence_threshold,
  const AddrRangeList _memory_ranges, const uint64_t _cache_block_size,
  const uint64_t _prefetch_distance, const uint64_t _prefetch_degree,
  const bool _can_cross_page, const Addr _page_size_in_bytes,
  const bool _stride_prefetch_pc_even_when_dmp_has_the_same_target_pc,
  DifferentialMatchingPrefetcherInterface *_prefetcher_interface
) : capacity(_capacity),
    confidence_threshold(_confidence_threshold),
    memory_ranges(_memory_ranges),
    cache_block_size(_cache_block_size),
    prefetch_distance(_prefetch_distance),
    prefetch_degree(_prefetch_degree),
    can_cross_page(_can_cross_page),
    page_size_in_bytes(_page_size_in_bytes),
    block_shift(log2(_cache_block_size)),
    page_shift(log2(_page_size_in_bytes)),
    stride_prefetch_pc_even_when_dmp_has_the_same_target_pc(
        _stride_prefetch_pc_even_when_dmp_has_the_same_target_pc
    ),
    prefetcher_interface(_prefetcher_interface),
    prefetch_queue(nullptr),
    recentPrefetchAddresses(64)
{
    stride_tracker.reserve(capacity);
    fatal_if(
        can_cross_page,
        "StrideTracker currently does not support generating prefetches that "
        "cross page boundaries."
    );
}

void
StrideTracker::setPrefetchQueue(PrefetchQueue *_prefetch_queue)
{
    prefetch_queue = _prefetch_queue;
}

std::optional<uint64_t>
StrideTracker::getAccessSizeForPC(const Addr pc) const
{
    for (const auto &entry : stride_tracker) {
        if (entry.pc == pc) {
            return entry.access_size;
        }
    }
    return std::nullopt;
}

void
StrideTracker::replaceLeastRecentlyUsedEntry(
    const Addr pc, const uint64_t access_size, const Addr paddr,
    const Tick access_timestamp
)
{
    if (stride_tracker.size() < capacity) {
        stride_tracker.emplace_back(
            pc, access_size, paddr, access_timestamp, confidence_threshold,
            prefetcher_interface
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
    const Addr evicted_pc = lru_it->pc;
    *lru_it = StrideTrackerEntry(
        pc, access_size, paddr, access_timestamp, confidence_threshold,
        prefetcher_interface
    );
    DMP_STRIDE_TRACKER_DEBUG(
        "Replaced LRU entry with new entry for PC %#x by evicting PC %#x\n",
        pc, evicted_pc
    );
}

void
StrideTracker::track(
  const Addr pc, const uint64_t access_size, const Addr paddr,
  const Tick access_timestamp
)
{
    // Check if the PC already exists in the stride tracker
    for (auto it = stride_tracker.begin(); it != stride_tracker.end(); ++it) {
        if (it->pc == pc) {
            // Found the entry, now we check confidence before updating
            const bool was_confident = it->isConfident();
            // Update the existing entry
            it->update(paddr, access_timestamp);
            // If we just recently became confident, notify the prefetcher
            if (!was_confident && it->isConfident()) {
                prefetcher_interface->handleNewlyDetectedStride(pc);
                DMP_STRIDE_TRACKER_DEBUG(
                    "PC %#x became confident with stride %ld\n",
                    pc, it->previous_stride
                );
            }
            // If the entry is confident, we emit prefetches
            if (it->isConfident()) {
                emitPrefetches(pc, paddr, access_timestamp);
            }
            return;
        }
    }

    // If not found, add a new entry (possibly replacing an old one)
    replaceLeastRecentlyUsedEntry(pc, access_size, paddr, access_timestamp);
}

void
StrideTracker::emitPrefetches(
  const Addr pc, const Addr current_paddr, const Tick access_timestamp
)
{
    // Find the entry for the given PC
    for (const auto &entry : stride_tracker) {
        if (entry.pc == pc && entry.isConfident()) {
            // If the DMP is already generating prefetches for this PC, we
            // skip emitting new prefetches to avoid interference.
            if (!stride_prefetch_pc_even_when_dmp_has_the_same_target_pc) {
                if (prefetcher_interface->isATargetPC(pc)) {
                    return;
                }
            }
            const int64_t stride = entry.previous_stride;
            for (
                uint64_t i = prefetch_distance;
                i <= prefetch_distance + prefetch_degree;
                ++i
            ) {
                const Addr prefetch_address = current_paddr + i * stride;
                bool is_out_of_bounds = true;
                for (const AddrRange &range : memory_ranges) {
                    if (range.contains(prefetch_address)) {
                        is_out_of_bounds = false;
                        break;
                    }
                }
                if (is_out_of_bounds) {
                    PrefetcherStats &stats = prefetcher_interface->getStats();
                    stats.numDMPPrefetchesDroppedDueToOutOfMemoryBounds++;
                    break;
                }
                if (recentPrefetchAddresses.contains(prefetch_address)) {
                    continue;
                }
                if (!can_cross_page) {
                    if (!samePage(current_paddr, prefetch_address)) {
                        break;
                    }
                } else {
                    // cross-page prefetching is currently not supported
                    fatal(
                        "StrideTracker does not support cross-page prefetching"
                    );
                }
                DMP_STRIDE_TRACKER_DEBUG(
                    "Emitting prefetch for PC %#x to address %#x\n",
                    pc, prefetch_address
                );
                // Check if the prefetch spans across cache blocks, and if so,
                // we currently don't emit such prefetches to avoid complexity.
                if (!sameBlock(
                    prefetch_address, prefetch_address + entry.access_size - 1
                )) {
                    PrefetcherStats &stats = prefetcher_interface->getStats();
                    stats.numStridePrefetchesDroppedDueToCrossBlockAccesses++;
                    DMP_STRIDE_TRACKER_DEBUG(
                        "Not emitting prefetch for PC %#x to address %#x as "
                        "it spans across cache blocks (access size %lu)\n",
                        pc, prefetch_address, entry.access_size
                    );
                    continue;
                }
                // Enqueue the prefetch request
                recentPrefetchAddresses.push(prefetch_address);
                prefetch_queue->enqueuePendingRequest(
                    PrefetchRequest(
                        /*_target_pc*/ pc,
                        /*_prefetch_vaddr*/ prefetch_address,
                        /*_size*/ entry.access_size,
                        /*_irt_id*/ 0
                    )
                );
                prefetcher_interface->getStats().numStridePrefetchesEmitted++;
            }
            break;
        }
    }
}

bool
StrideTracker::samePage(const Addr addr1, const Addr addr2) const
{
    return (addr1 >> page_shift) == (addr2 >> page_shift);
}

bool
StrideTracker::sameBlock(const Addr addr1, const Addr addr2) const
{
    return (addr1 >> block_shift) == (addr2 >> block_shift);
}

} // namespace dmp

} // namespace prefetch

} // namespace gem5
