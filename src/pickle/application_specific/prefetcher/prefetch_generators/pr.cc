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

#include "pickle/application_specific/prefetcher/prefetch_generators/pr.hh"

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/PickleDevicePrefetcherTrace.hh"
#include "debug/PickleDevicePrefetcherWorkTrackerDebug.hh"
#include "mem/packet.hh"
#include "pickle/application_specific/prefetcher/pickle_prefetcher.hh"
#include "pickle/application_specific/prefetcher/prefetcher_work_tracker.hh"

namespace gem5
{

PRPrefetchGenerator::PRPrefetchGenerator(
    std::string _name,
    const uint64_t _software_hint_distance,
    const uint64_t _prefetch_distance_offset_from_software_hint,
    PrefetcherWorkTracker* _work_tracker
) : PrefetchGenerator(
        _name,
        _software_hint_distance, _prefetch_distance_offset_from_software_hint,
        _work_tracker
    )
{
}

std::shared_ptr<WorkItem>
PRPrefetchGenerator::generateWorkItem(Addr work_data)
{
    // work_data is the address of the node that the core is working on

    const uint64_t node_id = work_data \
        + software_hint_distance - prefetch_distance_offset_from_software_hint;

    std::shared_ptr<WorkItem> workItem(new WorkItem(node_id));

    constexpr Addr BLOCK_SHIFT = 6;
    uint64_t lv1_start_edge_vaddr = 0;
    uint64_t lv1_end_edge_vaddr = 0;
    std::vector<uint64_t> lv2_edge_indices;

    // 0 -> 1 -> 2

    // level 1: we fetch the start and the end of the edge indices
    {
        bool success = false;
        Addr first_item_index = node_id;
        Addr array_vaddr = \
            work_tracker->job_descriptor->get_array(0).vaddr_start;

        Addr first_item_vaddr = array_vaddr + first_item_index * 8;
        Addr first_item_vaddr_block_aligned = \
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
                "pf_vaddr = 0x%llx, start_node\n",
                node_id, first_item_vaddr_block_aligned
            );
            return nullptr;
        }
        constexpr Addr item_size = 8;
        Addr start_index = \
            (first_item_vaddr - first_item_vaddr_block_aligned) / item_size;
        lv1_start_edge_vaddr = (pkt->getConstPtr<uint64_t>()[start_index]);
        // We add expected prefetches
        workItem->addExpectedPrefetch(first_item_vaddr_block_aligned, 0);
        warnIfOutsideRanges(node_id, first_item_vaddr_block_aligned);
        DPRINTF(
            PickleDevicePrefetcherTrace,
            "Work Item = 0x%llx, edge_start_ptr = 0x%llx\n",
            node_id, lv1_start_edge_vaddr
        );
    }
    {
        bool success = false;
        Addr second_item_index = node_id + 1;
        Addr array_vaddr = \
            work_tracker->job_descriptor->get_array(0).vaddr_start;

        Addr second_item_vaddr = array_vaddr + second_item_index * 8;
        Addr second_item_vaddr_block_aligned = \
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
                "pf_vaddr = 0x%llx, end_node\n",
                node_id, second_item_vaddr_block_aligned
            );
            return nullptr;
        }
        constexpr Addr item_size = 8;
        Addr end_index = \
            (second_item_vaddr - second_item_vaddr_block_aligned) / item_size;
        lv1_end_edge_vaddr = (pkt->getConstPtr<uint64_t>()[end_index]);
        // We add expected prefetches
        workItem->addExpectedPrefetch(second_item_vaddr_block_aligned, 0);
        warnIfOutsideRanges(node_id, second_item_vaddr_block_aligned);
        DPRINTF(
            PickleDevicePrefetcherTrace,
            "Work Item = 0x%llx, edge_end_ptr = 0x%llx\n",
            node_id, lv1_end_edge_vaddr
        );
    }
    // level 2: we fetch the edge indices
    {
        Addr curr_block_vaddr = 1;
        PacketPtr pkt = nullptr;
        uint32_t* data_ptr = nullptr;
        lv2_edge_indices.reserve(
            (lv1_end_edge_vaddr - lv1_start_edge_vaddr) / 4
        );
        for (
            Addr edge_vaddr = lv1_start_edge_vaddr;
            edge_vaddr < lv1_end_edge_vaddr;
            edge_vaddr += 4
        )
        {
            Addr edge_vaddr_block_aligned = \
                (edge_vaddr >> BLOCK_SHIFT) << BLOCK_SHIFT;
            if (edge_vaddr_block_aligned != curr_block_vaddr) {
                bool success = false;
                DPRINTF(
                    PickleDevicePrefetcherWorkTrackerDebug,
                    "Fetching lv2 vaddr 0x%llx\n",
                    edge_vaddr_block_aligned
                );
                pkt = work_tracker->owner->zeroCycleLoadWithVAddr(
                    edge_vaddr_block_aligned, success
                );
                if (!success) {
                    DPRINTF(
                        PickleDevicePrefetcherTrace,
                        "Failed to fetch level = 2, Work Item = 0x%llx, "
                        "vaddr = 0x%llx\n",
                        node_id, edge_vaddr_block_aligned
                    );
                    return nullptr;
                }
                curr_block_vaddr = edge_vaddr_block_aligned;
                data_ptr = pkt->getPtr<uint32_t>();
                // We add expected prefetches
                workItem->addExpectedPrefetch(curr_block_vaddr, 1);
                warnIfOutsideRanges(node_id, curr_block_vaddr);
            }
            constexpr Addr item_size = 4;
            Addr edge_index = \
                (edge_vaddr - curr_block_vaddr) / item_size;
            lv2_edge_indices.push_back(data_ptr[edge_index]);
            DPRINTF(
                PickleDevicePrefetcherTrace,
                "Work Item = 0x%llx, edge_index = %lld\n",
                node_id, lv2_edge_indices.back()
            );
        }
    }

    // level 3: we fetch the contrib array
    {
        Addr visited_start_vaddr = \
            work_tracker->job_descriptor->get_array(2).vaddr_start;
        for (auto edge_index : lv2_edge_indices) {
            Addr visited_vaddr = visited_start_vaddr + edge_index * 4;
            Addr visited_vaddr_block_aligned = \
                (visited_vaddr >> BLOCK_SHIFT) << BLOCK_SHIFT;
            // We add expected prefetches
            workItem->addExpectedPrefetch(visited_vaddr_block_aligned, 2);
            warnIfOutsideRanges(node_id, visited_vaddr_block_aligned);
            DPRINTF(
                PickleDevicePrefetcherTrace,
                "Work Item = 0x%llx, visited = 0x%llx\n",
                node_id, visited_vaddr_block_aligned
            );
        }
    }

    return workItem;
}

} // namespace gem5
