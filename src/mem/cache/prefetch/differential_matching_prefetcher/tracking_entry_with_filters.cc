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

#include "mem/cache/prefetch/differential_matching_prefetcher/tracking_entry_with_filters.hh"

#include <cstdint>

#include "base/types.hh"

namespace gem5
{

namespace prefetch
{

namespace dmp
{

TrackingEntryWithRepetitionFilter::TrackingEntryWithRepetitionFilter(
    const Addr _pc,
    const uint64_t _max_num_tracked_items
) : pc(_pc),
    max_num_tracked_items(_max_num_tracked_items),
    previous_update_tick(curTick())
{
    tracked_items.reserve(max_num_tracked_items);
}

void
TrackingEntryWithRepetitionFilter::addItem(
    const Addr item, const uint64_t size
)
{
    previous_update_tick = curTick();
    // If we have already tracked the maximum number of items, we do not add
    // more
    if (tracked_items.size() >= max_num_tracked_items) {
        return;
    }
    if (!tracked_items.empty()) {
        // Check for repetition with the last tracked item
        if (tracked_items.back().first == item) {
            return; // Do not add repeated item
        }
    }
    tracked_items.emplace_back(item, size);
}

bool
TrackingEntryWithRepetitionFilter::isFull() const
{
    return tracked_items.size() >= max_num_tracked_items;
}

TrackingEntryWithRepetitionFilterAndRangeFilter::\
    TrackingEntryWithRepetitionFilterAndRangeFilter (
    const Addr _pc,
    const uint64_t _max_num_tracked_items
) : pc(_pc),
    max_num_tracked_items(_max_num_tracked_items),
    previous_tracked_item(0),
    previous_size(0),
    previous_update_tick(curTick())
{
    tracked_items.reserve(max_num_tracked_items);
}

void
TrackingEntryWithRepetitionFilterAndRangeFilter::addItem(
    const Addr item, const uint64_t size
)
{
    previous_update_tick = curTick();
    // If we have already tracked the maximum number of items, we do not add
    // more
    if (tracked_items.size() >= max_num_tracked_items) {
        return;
    }
    if (!tracked_items.empty()) {
        // If the new item is exactly the same as the previous tracked item,
        // we do not add it (repetition filter)
        if (tracked_items.back().first == item) {
            return;
        }
        // If the new item is exactly after the previous tracked item, we
        // update the size of the last tracked item (range filter)
        if (previous_tracked_item + previous_size == item) {
            // The previous_size should be equal to size
            // assert(previous_size == size);
            // Increase the range size of the last tracked item
            tracked_items.back().second += 1;
            previous_tracked_item = item;
            return;
        }
    }
    tracked_items.emplace_back(item, 1);
    previous_tracked_item = item;
    previous_size = size;
}

bool
TrackingEntryWithRepetitionFilterAndRangeFilter::isFull() const
{
    return tracked_items.size() >= max_num_tracked_items;
}

} // namespace dmp

} // namespace prefetch

} // namespace gem5
