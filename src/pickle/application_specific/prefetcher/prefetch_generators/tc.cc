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

#include "pickle/application_specific/prefetcher/prefetch_generators/tc.hh"

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/PickleDevicePrefetcherTrace.hh"
#include "debug/PickleDevicePrefetcherWorkTrackerDebug.hh"
#include "mem/packet.hh"
#include "pickle/application_specific/prefetcher/pickle_prefetcher.hh"
#include "pickle/application_specific/prefetcher/prefetcher_work_tracker.hh"

namespace gem5
{

TCPrefetchGenerator::TCPrefetchGenerator(
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
TCPrefetchGenerator::generateWorkItem(Addr work_data)
{
    // work_data is the node that the core is working on
    const Addr node_id = work_data + \
        software_hint_distance - prefetch_distance_offset_from_software_hint;

    const uint64_t num_nodes = \
        work_tracker->job_descriptor->get_array(0).num_elements() - 1;

    DPRINTF(
        PickleDevicePrefetcherTrace,
        "Received work_data: 0x%llx, node_id: 0x%llx, num_nodes: 0x%llx\n",
        work_data, node_id, num_nodes
    );

    if (node_id >= num_nodes) {
        return nullptr;
    }

    std::shared_ptr<WorkItem> workItem(new WorkItem(node_id));

    // Level 1: we fetch the start_ptr/end_ptr from out_index_ptr array
    // Level 2: we fetch the outgoing neighbors from out_neighbors array

    // The array ids are as follows:
    // out_index_ptr (array_id 0)
    //     -> out_neighbors (array_id 1)

    const Addr out_index_ptr_base = \
        work_tracker->job_descriptor->get_array(0).vaddr_start;
    const uint64_t out_index_ptr_element_size = \
        work_tracker->job_descriptor->get_array(0).element_size;
    assert(out_index_ptr_element_size == 8);
    const Addr out_neighbors_base = \
        work_tracker->job_descriptor->get_array(1).vaddr_start;
    const uint64_t out_neighbors_element_size = \
        work_tracker->job_descriptor->get_array(1).element_size;
    assert(out_neighbors_element_size == 4);

    constexpr Addr BLOCK_SHIFT = 6;
    uint64_t lv1_start_edge_vaddr = 0;
    uint64_t lv1_end_edge_vaddr = 0;
    std::vector<uint64_t> lv2_edge_indices;

    // 0 -> 1

   // Level 1: we fetch the start_ptr/end_ptr from out_index_ptr array
    {
        bool success = false;

        const Addr start_ptr_vaddr = \
            out_index_ptr_base + node_id * out_index_ptr_element_size;
        const Addr start_ptr_vaddr_block_aligned = \
            (start_ptr_vaddr >> BLOCK_SHIFT) << BLOCK_SHIFT;
        DPRINTF(
            PickleDevicePrefetcherWorkTrackerDebug,
            "Fetching lv1 vaddr 0x%llx\n",
            start_ptr_vaddr_block_aligned
        );
        PacketPtr pkt = \
            work_tracker->owner->zeroCycleLoadWithVAddr(
                start_ptr_vaddr_block_aligned, success
            );
        if (!success) {
            DPRINTF(
                PickleDevicePrefetcherTrace,
                "Failed to fetch level = 1, Work Item = 0x%llx, "
                "pf_vaddr = 0x%llx, start_ptr\n",
                node_id, start_ptr_vaddr_block_aligned
            );
            return nullptr;
        }
        const Addr start_index = \
            (start_ptr_vaddr - start_ptr_vaddr_block_aligned) \
                / out_index_ptr_element_size;
        lv1_start_edge_vaddr = (pkt->getConstPtr<uint64_t>()[start_index]);
        // We add expected prefetches
        workItem->addExpectedPrefetch(start_ptr_vaddr_block_aligned, 0);
        warnIfOutsideRanges(node_id, start_ptr_vaddr_block_aligned);
        DPRINTF(
            PickleDevicePrefetcherTrace,
            "Work Item = 0x%llx, edge_start_ptr = 0x%llx\n",
            node_id, lv1_start_edge_vaddr
        );
    }
    {
        bool success = false;
        const Addr end_ptr_vaddr = \
            out_index_ptr_base + (node_id + 1) * out_index_ptr_element_size;
        const Addr end_ptr_vaddr_block_aligned = \
            (end_ptr_vaddr >> BLOCK_SHIFT) << BLOCK_SHIFT;
        DPRINTF(
            PickleDevicePrefetcherWorkTrackerDebug,
            "Fetching lv1 vaddr 0x%llx\n",
            end_ptr_vaddr_block_aligned
        );
        PacketPtr pkt = \
            work_tracker->owner->zeroCycleLoadWithVAddr(
                end_ptr_vaddr_block_aligned, success
            );
        if (!success) {
            DPRINTF(
                PickleDevicePrefetcherTrace,
                "Failed to fetch level = 1, Work Item = 0x%llx, "
                "pf_vaddr = 0x%llx, end_ptr\n",
                node_id, end_ptr_vaddr_block_aligned
            );
            return nullptr;
        }
        const Addr end_index = \
            (end_ptr_vaddr - end_ptr_vaddr_block_aligned) \
                / out_index_ptr_element_size;
        lv1_end_edge_vaddr = (pkt->getConstPtr<uint64_t>()[end_index]);
        // We add expected prefetches
        workItem->addExpectedPrefetch(end_ptr_vaddr_block_aligned, 0);
        warnIfOutsideRanges(node_id, end_ptr_vaddr_block_aligned);
        DPRINTF(
            PickleDevicePrefetcherTrace,
            "Work Item = 0x%llx, edge_end_ptr = 0x%llx\n",
            node_id, lv1_end_edge_vaddr
        );
    }
    // Level 2: we fetch the outgoing neighbors from out_neighbors array
    {
        Addr curr_block_vaddr = 1;
        PacketPtr pkt = nullptr;
        uint32_t* data_ptr = nullptr;
        lv2_edge_indices.reserve(
            (lv1_end_edge_vaddr - lv1_start_edge_vaddr) \
                / out_neighbors_element_size
        );
        for (
            Addr edge_vaddr = lv1_start_edge_vaddr;
            edge_vaddr < lv1_end_edge_vaddr;
            edge_vaddr += out_neighbors_element_size
        )
        {
            const Addr edge_vaddr_block_aligned = \
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
            const Addr edge_index = \
                (edge_vaddr - curr_block_vaddr) / out_neighbors_element_size;
            lv2_edge_indices.push_back(data_ptr[edge_index]);
            DPRINTF(
                PickleDevicePrefetcherTrace,
                "Work Item = 0x%llx, edge_index = %lld\n",
                node_id, lv2_edge_indices.back()
            );
        }
    }

    return workItem;
}

} // namespace gem5
