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

#ifndef __DMP_TRACKING_ENTRY_WITH_FILTERS_HH__
#define __DMP_TRACKING_ENTRY_WITH_FILTERS_HH__

#include <array>
#include <cstdint>
#include <vector>

#include "base/types.hh"

namespace gem5
{

namespace prefetch
{

class TrackingEntryWithRepetitionFilter
{
  public:
    Addr pc;
    uint64_t max_num_tracked_items;
    // Tracked items: pair of <item, size>
    // For index PC, item is data value, and size is data size (in bytes)
    std::vector<std::pair<Addr, uint64_t>> tracked_items;
    TrackingEntryWithRepetitionFilter(
      const Addr _pc, const uint64_t _max_num_tracked_items
    );
    void addItem(const Addr item, const uint64_t size);
    bool isFull() const;
}; // class TrackingEntryWithRepetitionFilter

// We track target PC accesses with both repetition filter and range filter
// E.g., (0x1000, size=4), (0x1000, size=4),(0x1004, size=4), (0x1008, size=4)
// will be stored as one entry: (0x1000, range_size=3, item_size=4)
class TrackingEntryWithRepetitionFilterAndRangeFilter
{
  public:
    Addr pc;
    uint64_t max_num_tracked_items;
    Addr previous_tracked_item;
    Addr previous_size;
    // Tracked items: pair of <item, size>
    // For target PC, the item is effective virtual address, and size is the
    // range size (in number of items).
    std::vector<std::pair<Addr, uint64_t>> tracked_items;
    TrackingEntryWithRepetitionFilterAndRangeFilter(
      const Addr _pc, const uint64_t _max_num_tracked_items
    );
    void addItem(const Addr item, const uint64_t size);
    bool isFull() const;
}; // class TrackingEntryWithRepetitionFilterAndRangeFilter

} // namespace prefetch

} // namespace gem5

#endif // __DMP_TRACKING_ENTRY_WITH_FILTERS_HH__
