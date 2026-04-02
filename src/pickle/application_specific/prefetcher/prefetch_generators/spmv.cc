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

#include "pickle/application_specific/prefetcher/prefetch_generators/spmv.hh"

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/PickleDevicePrefetcherTrace.hh"
#include "debug/PickleDevicePrefetcherWorkTrackerDebug.hh"
#include "mem/packet.hh"
#include "pickle/application_specific/prefetcher/pickle_prefetcher.hh"
#include "pickle/application_specific/prefetcher/prefetcher_work_tracker.hh"

namespace gem5
{

SPMVPrefetchGenerator::SPMVPrefetchGenerator(
    std::string _name,
    const uint64_t _job_id, const uint64_t _core_id,
    const uint64_t _software_hint_distance,
    const uint64_t _prefetch_distance_offset_from_software_hint,
    const uint64_t _max_requests_per_level,
    PrefetcherWorkTracker* _work_tracker
) : PrefetchGenerator(
    _name,
    _job_id, _core_id,
    _software_hint_distance, _prefetch_distance_offset_from_software_hint,
    _max_requests_per_level,
    _work_tracker
    )
{
}

std::shared_ptr<WorkItem>
SPMVPrefetchGenerator::execute_kernel(Addr work_data)
{
    // work_data is the node_id that the core is working on
    const Addr work_id = work_data + software_hint_distance - \
        prefetch_distance_offset_from_software_hint;

    // Level 1: we fetch the start and the end of the work_id's row
    // Level 2: we fetch the column indices of the row
    // Level 3: we fetch the values of x array

    // The array ids are as follows:
    // row (array 0) -> col_ind (array_id 1) -> x (array_id 3)

    const uint64_t num_rows = \
        work_tracker->job_descriptor->get_array(0).num_elements() - 1;
    //const uint64_t num_cols = \
    //    work_tracker->job_descriptor->get_array(3).num_elements();
    if (work_id >= num_rows) {
        return nullptr;
    }

    const uint64_t node_id = work_id;
    // results from level 1 prefetches
    uint64_t row_start = 0;
    uint64_t row_end = 0;
    // results from level 2 prefetches
    std::vector<uint64_t> col_indices;

    const Addr row_ptr_base_vaddr = \
        work_tracker->job_descriptor->get_array(0).vaddr_start;
    const uint64_t row_ptr_element_size = \
        work_tracker->job_descriptor->get_array(0).element_size;
    assert(row_ptr_element_size == 4);
    const Addr col_ind_base_vaddr = \
        work_tracker->job_descriptor->get_array(1).vaddr_start;
    const uint64_t col_ind_element_size = \
        work_tracker->job_descriptor->get_array(1).element_size;
    assert(row_ptr_element_size == 4);
    const Addr x_base_vaddr = \
        work_tracker->job_descriptor->get_array(3).vaddr_start;
    const uint64_t x_element_size = \
        work_tracker->job_descriptor->get_array(3).element_size;

    std::shared_ptr<WorkItem> workItem(new WorkItem(work_id));

    constexpr Addr BLOCK_SHIFT = 6;

    DPRINTF(
        PickleDevicePrefetcherWorkTrackerDebug,
        "----- Working on node_id %lld\n", node_id
    );
    // level 1: we fetch the start and the end of the work_id's row
    {
        bool success = false;

        const Addr first_item_vaddr = row_ptr_base_vaddr + \
            node_id * row_ptr_element_size;
        const Addr first_item_vaddr_block_aligned = \
            (first_item_vaddr >> BLOCK_SHIFT) << BLOCK_SHIFT;
        DPRINTF(
            PickleDevicePrefetcherWorkTrackerDebug,
            "Fetching lv1 vaddr 0x%llx\n",
            first_item_vaddr_block_aligned
        );
        PacketPtr pkt = \
            work_tracker->owner->zeroCycleLoadWithVAddr(
                first_item_vaddr_block_aligned, success
            );
        if (!success) {
            DPRINTF(
                PickleDevicePrefetcherTrace,
                "Failed to fetch level = 1, Work Item = 0x%llx, "
                "pf_vaddr = 0x%llx, row_start\n",
                work_id, first_item_vaddr_block_aligned
            );
            return nullptr;
        }
        const Addr start_index = \
            (first_item_vaddr - first_item_vaddr_block_aligned) \
                / row_ptr_element_size;
        // // 4 bytes per element
        row_start = (pkt->getConstPtr<uint32_t>()[start_index]);
        // We add expected prefetches
        workItem->addExpectedPrefetch(first_item_vaddr_block_aligned, 0);
        warnIfOutsideRanges(work_id, first_item_vaddr_block_aligned);
        DPRINTF(
            PickleDevicePrefetcherTrace,
            "Work Item = 0x%llx, row_start = 0x%llx\n",
            work_id, row_start
        );
    }
    {
        bool success = false;

        const Addr second_item_vaddr = row_ptr_base_vaddr + \
            (node_id + 1) * row_ptr_element_size;
        const Addr second_item_vaddr_block_aligned = \
            (second_item_vaddr >> BLOCK_SHIFT) << BLOCK_SHIFT;
        DPRINTF(
            PickleDevicePrefetcherWorkTrackerDebug,
            "Fetching lv1 vaddr 0x%llx\n",
            second_item_vaddr_block_aligned
        );
        PacketPtr pkt = \
            work_tracker->owner->zeroCycleLoadWithVAddr(
                second_item_vaddr_block_aligned, success
            );
        if (!success) {
            DPRINTF(
                PickleDevicePrefetcherTrace,
                "Failed to fetch level = 1, Work Item = 0x%llx, "
                "pf_vaddr = 0x%llx, row_end\n",
                work_id, second_item_vaddr_block_aligned
            );
            return nullptr;
        }
        const Addr end_index = \
            (second_item_vaddr - second_item_vaddr_block_aligned) \
                / row_ptr_element_size;
        // // 4 bytes per element
        row_end = (pkt->getConstPtr<uint32_t>()[end_index]);
        // We add expected prefetches
        workItem->addExpectedPrefetch(second_item_vaddr_block_aligned, 0);
        warnIfOutsideRanges(work_id, second_item_vaddr_block_aligned);
        DPRINTF(
            PickleDevicePrefetcherTrace,
            "Work Item = 0x%llx, row_end = 0x%llx\n",
            work_id, row_end
        );
    }
    if (row_start == row_end) {
        // empty row
        return workItem;
    }

    // level 2: we fetch the column indices of the row
    {
        Addr curr_block_vaddr = 1;
        PacketPtr pkt = nullptr;
        uint32_t* data_ptr = nullptr;
        const Addr start_col_vaddr = \
            col_ind_base_vaddr + row_start * col_ind_element_size;
        const Addr end_col_vaddr = \
            col_ind_base_vaddr + (row_end - 1) * col_ind_element_size;

        DPRINTF(
            PickleDevicePrefetcherWorkTrackerDebug,
            "row_start: %lld, row_end: %lld\n",
            row_start, row_end
        );
        col_indices.reserve(row_end - row_start);
        for (
            Addr col_vaddr = start_col_vaddr;
            col_vaddr <= end_col_vaddr;
            col_vaddr += 4
        )
        {
            const Addr col_vaddr_block_aligned = \
                (col_vaddr >> BLOCK_SHIFT) << BLOCK_SHIFT;
            if (col_vaddr_block_aligned != curr_block_vaddr) {
                bool success = false;
                DPRINTF(
                    PickleDevicePrefetcherWorkTrackerDebug,
                    "Fetching col vaddr 0x%llx\n",
                    col_vaddr_block_aligned
                );
                pkt = work_tracker->owner->zeroCycleLoadWithVAddr(
                    col_vaddr_block_aligned, success
                );
                if (!success) {
                    DPRINTF(
                        PickleDevicePrefetcherTrace,
                        "Failed to fetch level = 2, Work Item = 0x%llx, "
                        "vaddr = 0x%llx\n",
                        work_id, col_vaddr_block_aligned
                    );
                    return nullptr;
                }
                curr_block_vaddr = col_vaddr_block_aligned;
                data_ptr = pkt->getPtr<uint32_t>();
                // We add expected prefetches
                workItem->addExpectedPrefetch(curr_block_vaddr, 1);
                warnIfOutsideRanges(work_id, curr_block_vaddr);
            }
            constexpr Addr item_size = 4;
            Addr col_index = \
                (col_vaddr - curr_block_vaddr) / item_size;
            col_indices.push_back(data_ptr[col_index]);
            DPRINTF(
                PickleDevicePrefetcherTrace,
                "Work Item = 0x%llx, col_index = %lld\n",
                work_id, col_indices.back()
            );
            if (col_indices.size() >= max_requests_per_level) {
                break;
            }
        }
    }

    // level 3: we fetch the values of x array
    {
        for (auto col_index : col_indices) {
            Addr x_element_vaddr = x_base_vaddr + col_index * x_element_size;
            Addr x_element_vaddr_block_aligned = \
                (x_element_vaddr >> BLOCK_SHIFT) << BLOCK_SHIFT;
            // We add expected prefetches
            workItem->addExpectedPrefetch(x_element_vaddr_block_aligned, 2);
            warnIfOutsideRanges(work_id, x_element_vaddr_block_aligned);
            DPRINTF(
                PickleDevicePrefetcherTrace,
                "Work Item = 0x%llx, x_element = 0x%llx\n",
                work_id, x_element_vaddr_block_aligned
            );
        }
    }

    return workItem;
}

} // namespace gem5
