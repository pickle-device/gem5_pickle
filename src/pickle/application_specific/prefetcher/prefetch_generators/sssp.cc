/*
 * Copyright (c) 2026 The Regents of the University of California
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

#include "pickle/application_specific/prefetcher/prefetch_generators/sssp.hh"

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/PickleDevicePrefetcherTrace.hh"
#include "debug/PickleDevicePrefetcherWorkTrackerDebug.hh"
#include "mem/packet.hh"
#include "pickle/application_specific/prefetcher/pickle_prefetcher.hh"
#include "pickle/application_specific/prefetcher/prefetcher_work_tracker.hh"

namespace gem5
{

SSSPPrefetchKernel1Generator::SSSPPrefetchKernel1Generator(
    std::string _name,
    const uint64_t _job_id, const uint64_t _core_id,
    const uint64_t _software_hint_distance,
    const uint64_t _prefetch_distance_offset_from_software_hint,
    PrefetcherWorkTracker* _work_tracker
) : PrefetchGenerator(
    _name,
    _job_id, _core_id,
    _software_hint_distance, _prefetch_distance_offset_from_software_hint,
    _work_tracker
    )
{
}

std::shared_ptr<WorkItem>
SSSPPrefetchKernel1Generator::execute_kernel(Addr work_data)
{
    // array 0: frontier
    // array 1: out_index
    // array 2: out_neighbors
    // array 3: dist
    // 0 -> 1 -> 2 -> 3

    // work_data is the address of the node that the core is working on
    const Addr work_id = \
        work_data + software_hint_distance * 4 - \
            prefetch_distance_offset_from_software_hint * 4;
    const Addr work_vaddr = work_id;

    std::shared_ptr<WorkItem> workItem(new WorkItem(work_id));
    PREFETCHER_WORK_TRACKER_DEBUG(
        "Received work item with vaddr 0x%llx\n", work_vaddr
    );

    constexpr Addr BLOCK_SHIFT = 6;
    uint64_t lv1_node_id = 0;
    uint64_t lv2_start_ptr_vaddr = 0;
    uint64_t lv2_end_ptr_vaddr = 0;
    std::vector<uint64_t> lv3_edge_indices;

    // level 1: we fetch the node id
    {
        bool success = false;
        const Addr block_aligned_vaddr =
            (work_vaddr >> BLOCK_SHIFT) << BLOCK_SHIFT;
        constexpr Addr item_size = 4;
        PREFETCHER_WORK_TRACKER_DEBUG(
            "Fetching lv1 vaddr 0x%llx\n", block_aligned_vaddr
        );
        PacketPtr pkt = \
            work_tracker->owner->zeroCycleLoadWithVAddr(
                block_aligned_vaddr, success
            );
        if (!success) {
            PREFETCHER_TRACE_DEBUG(
                "Failed to fetch level = 1, vaddr = 0x%llx\n", work_vaddr
            );
            return nullptr;
        }
        Addr node_id_offset = (work_vaddr - block_aligned_vaddr) / item_size;
        lv1_node_id = \
            (uint64_t)(pkt->getConstPtr<uint32_t>()[node_id_offset]);
        // We add expected prefetches
        workItem->addExpectedPrefetch(block_aligned_vaddr, 0);
        warnIfOutsideRanges(work_vaddr, block_aligned_vaddr);
        PREFETCHER_TRACE_DEBUG(
            "Work Item = 0x%llx, node_id = %lld\n", work_vaddr, lv1_node_id
        );
    }
    // level 1.1: we fetch dist[u] to decide if we want to prefetch the
    // neighbor list of the node or not
    {
        // safe guard: This is not a problem but we want to make sure that the
        // prefetch vaddr is within the valid range of the job's memory.
        const uint64_t dist_array_num_elements =
            work_tracker->job_descriptor->get_array(3).num_elements();
        if (lv1_node_id >= dist_array_num_elements) {
            PREFETCHER_TRACE_DEBUG(
                "Node id %lld is out of bounds for dist array with %lld "
                "elements. Work Item = 0x%llx\n",
                lv1_node_id, dist_array_num_elements, work_vaddr
            );
            return nullptr;
        }

        bool success = false;
        const Addr dist_vaddr = \
            work_tracker->job_descriptor->get_array(3).vaddr_start + \
                lv1_node_id * 4;
        const Addr dist_vaddr_block_aligned = \
            (dist_vaddr >> BLOCK_SHIFT) << BLOCK_SHIFT;
        PREFETCHER_WORK_TRACKER_DEBUG(
            "Fetching dist[u] vaddr 0x%llx\n", dist_vaddr_block_aligned
        );
        PacketPtr pkt = \
            work_tracker->owner->zeroCycleLoadWithVAddr(
                dist_vaddr_block_aligned, success
            );
        if (!success) {
            PREFETCHER_TRACE_DEBUG(
                "Failed to fetch dist[u], Work Item = 0x%llx, "
                "pf_vaddr = 0x%llx\n",
                work_vaddr, dist_vaddr_block_aligned
            );
            return nullptr;
        }
        constexpr Addr item_size = 4;
        const Addr dist_index =
            (dist_vaddr - dist_vaddr_block_aligned) / item_size;
        const uint32_t dist_value = pkt->getConstPtr<uint32_t>()[dist_index];
        // We add expected prefetches
        workItem->addExpectedPrefetch(dist_vaddr_block_aligned, 1);
        warnIfOutsideRanges(work_vaddr, dist_vaddr_block_aligned);
        PREFETCHER_TRACE_DEBUG(
            "Work Item = 0x%llx, dist[%lld] = %u\n",
            work_vaddr, lv1_node_id, dist_value
        );
        // We only prefetch the neighbor list if the distance is bigger than
        // the threshold
        const uint32_t distance_threshold = \
             prefetch_context->getSSSPCurrentDistanceThreshold(core_id);
        if (dist_value < distance_threshold) {
            // dist below threshold, skip prefetching neighbors
            PREFETCHER_TRACE_DEBUG(
                "Distance %u is less than threshold %u, skip prefetching "
                "neighbors for node %lld\n",
                dist_value,
                distance_threshold,
                lv1_node_id
            );
            return workItem;
        }
    }
    // level 2: we fetch the start pointer and the end pointer of the neighbor
    // edge list
    {
        bool success = false;
        const Addr first_item_index = lv1_node_id;
        const Addr array_vaddr = \
            work_tracker->job_descriptor->get_array(1).vaddr_start;

        const Addr first_item_vaddr = array_vaddr + first_item_index * 8;
        const Addr first_item_vaddr_block_aligned = \
            (first_item_vaddr >> BLOCK_SHIFT) << BLOCK_SHIFT;
        PREFETCHER_WORK_TRACKER_DEBUG(
            "Fetching lv2 vaddr 0x%llx\n", first_item_vaddr_block_aligned
        );
        PacketPtr pkt = \
            work_tracker->owner->zeroCycleLoadWithVAddr(
                first_item_vaddr_block_aligned, success
            );
        if (!success) {
            PREFETCHER_TRACE_DEBUG(
                "Failed to fetch level = 2, Work Item = 0x%llx, "
                "pf_vaddr = 0x%llx, start_node\n",
                work_vaddr, first_item_vaddr_block_aligned
            );
            return nullptr;
        }
        constexpr Addr item_size = 8;
        const Addr start_index = \
            (first_item_vaddr - first_item_vaddr_block_aligned) / item_size;
        lv2_start_ptr_vaddr = (pkt->getConstPtr<uint64_t>()[start_index]);
        // We add expected prefetches
        workItem->addExpectedPrefetch(first_item_vaddr_block_aligned, 2);
        warnIfOutsideRanges(work_vaddr, first_item_vaddr_block_aligned);
        PREFETCHER_TRACE_DEBUG(
            "Work Item = 0x%llx, edge_start_ptr = 0x%llx\n",
            work_vaddr, lv2_start_ptr_vaddr
        );
    }
    {
        bool success = false;
        const Addr second_item_index = lv1_node_id + 1;
        const Addr array_vaddr = \
            work_tracker->job_descriptor->get_array(1).vaddr_start;
        const Addr second_item_vaddr = array_vaddr + second_item_index * 8;
        const Addr second_item_vaddr_block_aligned = \
            (second_item_vaddr >> BLOCK_SHIFT) << BLOCK_SHIFT;
        PREFETCHER_WORK_TRACKER_DEBUG(
            "Fetching lv2 vaddr 0x%llx\n", second_item_vaddr_block_aligned
        );
        PacketPtr pkt = \
            work_tracker->owner->zeroCycleLoadWithVAddr(
                second_item_vaddr_block_aligned, success
            );
        if (!success) {
            PREFETCHER_TRACE_DEBUG(
                "Failed to fetch level = 2, Work Item = 0x%llx, "
                "pf_vaddr = 0x%llx, end_node\n",
                work_vaddr, second_item_vaddr_block_aligned
            );
            return nullptr;
        }
        constexpr Addr item_size = 8;
        const Addr end_index = \
            (second_item_vaddr - second_item_vaddr_block_aligned) / item_size;
        lv2_end_ptr_vaddr = (pkt->getConstPtr<uint64_t>()[end_index]);
        // We add expected prefetches
        workItem->addExpectedPrefetch(second_item_vaddr_block_aligned, 2);
        warnIfOutsideRanges(work_vaddr, second_item_vaddr_block_aligned);
        PREFETCHER_TRACE_DEBUG(
            "Work Item = 0x%llx, edge_end_ptr = 0x%llx\n",
            work_vaddr, lv2_end_ptr_vaddr
        );
    }
    // level 3: we fetch the edge indices and its weights
    // Different from other workloads, the out_neighbor array of SSSP contain
    // both the edge indices and their weights. Specifically, for each edge, we
    // first have the edge index (4-byte int), and then we have the edge weight
    // (also 4-byte int).
    {
        Addr curr_block_vaddr = 1;
        PacketPtr pkt = nullptr;
        uint64_t* data_ptr = nullptr;
        lv3_edge_indices.reserve(
            (lv2_end_ptr_vaddr - lv2_start_ptr_vaddr) / 8
        );
        for (
            Addr edge_vaddr = lv2_start_ptr_vaddr;
            edge_vaddr < lv2_end_ptr_vaddr;
            edge_vaddr += 8
        )
        {
            const Addr edge_vaddr_block_aligned = \
                (edge_vaddr >> BLOCK_SHIFT) << BLOCK_SHIFT;
            if (edge_vaddr_block_aligned != curr_block_vaddr) {
                bool success = false;
                PREFETCHER_WORK_TRACKER_DEBUG(
                    "Fetching lv3 vaddr 0x%llx\n", edge_vaddr_block_aligned
                );
                pkt = work_tracker->owner->zeroCycleLoadWithVAddr(
                    edge_vaddr_block_aligned, success
                );
                if (!success) {
                    PREFETCHER_TRACE_DEBUG(
                        "Failed to fetch level = 3, Work Item = 0x%llx, "
                        "vaddr = 0x%llx\n",
                        work_vaddr, edge_vaddr_block_aligned
                    );
                    return nullptr;
                }
                curr_block_vaddr = edge_vaddr_block_aligned;
                data_ptr = pkt->getPtr<uint64_t>();
                // We add expected prefetches
                workItem->addExpectedPrefetch(curr_block_vaddr, 3);
                warnIfOutsideRanges(work_vaddr, curr_block_vaddr);
            }
            constexpr Addr item_size = 8;
            const Addr index_and_weight = \
                (edge_vaddr - curr_block_vaddr) / item_size;
            constexpr uint64_t mask_of_32_bits = (1ULL << 32) - 1;
            const uint64_t edge_index = index_and_weight & mask_of_32_bits;
            const uint64_t edge_weight =
                (index_and_weight >> 32) & mask_of_32_bits;
            lv3_edge_indices.push_back(data_ptr[edge_index]);
            PREFETCHER_TRACE_DEBUG(
                "Work Item = 0x%llx, edge_index = %lld, edge_weight = %lld\n",
                work_vaddr, lv3_edge_indices.back(), edge_weight
            );
        }
    }

    // level 4: we fetch the dist array
    {
        const Addr dist_array_start_vaddr = \
            work_tracker->job_descriptor->get_array(3).vaddr_start;
        for (auto edge_index : lv3_edge_indices) {
            // dist array
            const Addr dist_vaddr =
                dist_array_start_vaddr + edge_index * 4;
            const Addr dist_vaddr_block_aligned = \
                (dist_vaddr >> BLOCK_SHIFT) << BLOCK_SHIFT;
            // We add expected prefetches
            workItem->addExpectedPrefetch(dist_vaddr_block_aligned, 4);
            warnIfOutsideRanges(work_vaddr, dist_vaddr_block_aligned);
            PREFETCHER_TRACE_DEBUG(
                "Work Item = 0x%llx, dist = 0x%llx\n",
                work_vaddr, dist_vaddr_block_aligned
            );
        }
    }

    return workItem;
}

SSSPPrefetchKernel2Generator::SSSPPrefetchKernel2Generator(
    std::string _name,
    const uint64_t _job_id, const uint64_t _core_id,
    const uint64_t _software_hint_distance,
    const uint64_t _prefetch_distance_offset_from_software_hint,
    PrefetcherWorkTracker* _work_tracker
) : PrefetchGenerator(
    _name,
    _job_id, _core_id,
    _software_hint_distance, _prefetch_distance_offset_from_software_hint,
    _work_tracker
    )
{
}

std::shared_ptr<WorkItem>
SSSPPrefetchKernel2Generator::execute_kernel(Addr work_data)
{
    // array 0: curr_bin_copy
    // array 1: out_index
    // array 2: out_neighbors
    // array 3: dist
    // 0 -> 1 -> 2 -> 3

    // work_data is the address of the node that the core is working on
    const Addr work_id = \
        work_data + software_hint_distance * 4 - \
            prefetch_distance_offset_from_software_hint * 4;
    const Addr work_vaddr = work_id;

    std::shared_ptr<WorkItem> workItem(new WorkItem(work_id));

    constexpr Addr BLOCK_SHIFT = 6;
    uint64_t lv1_node_id = 0;
    uint64_t lv2_start_ptr_vaddr = 0;
    uint64_t lv2_end_ptr_vaddr = 0;
    std::vector<uint64_t> lv3_edge_indices;

    // level 1: we fetch the node id
    {
        bool success = false;
        const Addr block_aligned_vaddr =
            (work_vaddr >> BLOCK_SHIFT) << BLOCK_SHIFT;
        constexpr Addr item_size = 4;
        PREFETCHER_WORK_TRACKER_DEBUG(
            "Fetching lv1 vaddr 0x%llx\n", block_aligned_vaddr
        );
        PacketPtr pkt = \
            work_tracker->owner->zeroCycleLoadWithVAddr(
                block_aligned_vaddr, success
            );
        if (!success) {
            PREFETCHER_TRACE_DEBUG(
                "Failed to fetch level = 1, vaddr = 0x%llx\n", work_vaddr
            );
            return nullptr;
        }
        Addr node_id_offset = (work_vaddr - block_aligned_vaddr) / item_size;
        lv1_node_id = \
            (uint64_t)(pkt->getConstPtr<uint32_t>()[node_id_offset]);
        // We add expected prefetches
        workItem->addExpectedPrefetch(block_aligned_vaddr, 0);
        warnIfOutsideRanges(work_vaddr, block_aligned_vaddr);
        PREFETCHER_TRACE_DEBUG(
            "Work Item = 0x%llx, node_id = %lld\n", work_vaddr, lv1_node_id
        );
    }
    // level 2: we fetch the start pointer and the end pointer of the neighbor
    // edge list
    {
        bool success = false;
        const Addr first_item_index = lv1_node_id;
        const Addr array_vaddr = \
            work_tracker->job_descriptor->get_array(1).vaddr_start;

        const Addr first_item_vaddr = array_vaddr + first_item_index * 8;
        const Addr first_item_vaddr_block_aligned = \
            (first_item_vaddr >> BLOCK_SHIFT) << BLOCK_SHIFT;
        PREFETCHER_WORK_TRACKER_DEBUG(
            "Fetching lv2 vaddr 0x%llx\n", first_item_vaddr_block_aligned
        );
        PacketPtr pkt = \
            work_tracker->owner->zeroCycleLoadWithVAddr(
                first_item_vaddr_block_aligned, success
            );
        if (!success) {
            PREFETCHER_TRACE_DEBUG(
                "Failed to fetch level = 2, Work Item = 0x%llx, "
                "pf_vaddr = 0x%llx, start_node\n",
                work_vaddr, first_item_vaddr_block_aligned
            );
            return nullptr;
        }
        constexpr Addr item_size = 8;
        const Addr start_index = \
            (first_item_vaddr - first_item_vaddr_block_aligned) / item_size;
        lv2_start_ptr_vaddr = (pkt->getConstPtr<uint64_t>()[start_index]);
        // We add expected prefetches
        workItem->addExpectedPrefetch(first_item_vaddr_block_aligned, 1);
        warnIfOutsideRanges(work_vaddr, first_item_vaddr_block_aligned);
        PREFETCHER_TRACE_DEBUG(
            "Work Item = 0x%llx, edge_start_ptr = 0x%llx\n",
            work_vaddr, lv2_start_ptr_vaddr
        );
    }
    {
        bool success = false;
        const Addr second_item_index = lv1_node_id + 1;
        const Addr array_vaddr = \
            work_tracker->job_descriptor->get_array(1).vaddr_start;
        const Addr second_item_vaddr = array_vaddr + second_item_index * 8;
        const Addr second_item_vaddr_block_aligned = \
            (second_item_vaddr >> BLOCK_SHIFT) << BLOCK_SHIFT;
        PREFETCHER_WORK_TRACKER_DEBUG(
            "Fetching lv2 vaddr 0x%llx\n", second_item_vaddr_block_aligned
        );
        PacketPtr pkt = \
            work_tracker->owner->zeroCycleLoadWithVAddr(
                second_item_vaddr_block_aligned, success
            );
        if (!success) {
            PREFETCHER_TRACE_DEBUG(
                "Failed to fetch level = 2, Work Item = 0x%llx, "
                "pf_vaddr = 0x%llx, end_node\n",
                work_vaddr, second_item_vaddr_block_aligned
            );
            return nullptr;
        }
        constexpr Addr item_size = 8;
        const Addr end_index = \
            (second_item_vaddr - second_item_vaddr_block_aligned) / item_size;
        lv2_end_ptr_vaddr = (pkt->getConstPtr<uint64_t>()[end_index]);
        // We add expected prefetches
        workItem->addExpectedPrefetch(second_item_vaddr_block_aligned, 1);
        warnIfOutsideRanges(work_vaddr, second_item_vaddr_block_aligned);
        PREFETCHER_TRACE_DEBUG(
            "Work Item = 0x%llx, edge_end_ptr = 0x%llx\n",
            work_vaddr, lv2_end_ptr_vaddr
        );
    }
    // level 3: we fetch the edge indices and its weights
    // Different from other workloads, the out_neighbor array of SSSP contain
    // both the edge indices and their weights. Specifically, for each edge, we
    // first have the edge index (4-byte int), and then we have the edge weight
    // (also 4-byte int).
    {
        Addr curr_block_vaddr = 1;
        PacketPtr pkt = nullptr;
        uint64_t* data_ptr = nullptr;
        lv3_edge_indices.reserve(
            (lv2_end_ptr_vaddr - lv2_start_ptr_vaddr) / 8
        );
        for (
            Addr edge_vaddr = lv2_start_ptr_vaddr;
            edge_vaddr < lv2_end_ptr_vaddr;
            edge_vaddr += 8
        )
        {
            const Addr edge_vaddr_block_aligned = \
                (edge_vaddr >> BLOCK_SHIFT) << BLOCK_SHIFT;
            if (edge_vaddr_block_aligned != curr_block_vaddr) {
                bool success = false;
                PREFETCHER_WORK_TRACKER_DEBUG(
                    "Fetching lv3 vaddr 0x%llx\n", edge_vaddr_block_aligned
                );
                pkt = work_tracker->owner->zeroCycleLoadWithVAddr(
                    edge_vaddr_block_aligned, success
                );
                if (!success) {
                    PREFETCHER_TRACE_DEBUG(
                        "Failed to fetch level = 3, Work Item = 0x%llx, "
                        "vaddr = 0x%llx\n",
                        work_vaddr, edge_vaddr_block_aligned
                    );
                    return nullptr;
                }
                curr_block_vaddr = edge_vaddr_block_aligned;
                data_ptr = pkt->getPtr<uint64_t>();
                // We add expected prefetches
                workItem->addExpectedPrefetch(curr_block_vaddr, 2);
                warnIfOutsideRanges(work_vaddr, curr_block_vaddr);
            }
            constexpr Addr item_size = 8;
            const Addr index_and_weight = \
                (edge_vaddr - curr_block_vaddr) / item_size;
            constexpr uint64_t mask_of_32_bits = (1ULL << 32) - 1;
            const uint64_t edge_index = index_and_weight & mask_of_32_bits;
            const uint64_t edge_weight =
                (index_and_weight >> 32) & mask_of_32_bits;
            lv3_edge_indices.push_back(data_ptr[edge_index]);
            PREFETCHER_TRACE_DEBUG(
                "Work Item = 0x%llx, edge_index = %lld, edge_weight = %lld\n",
                work_vaddr, lv3_edge_indices.back(), edge_weight
            );
        }
    }

    // level 4: we fetch the dist array
    {
        const Addr dist_array_start_vaddr = \
            work_tracker->job_descriptor->get_array(3).vaddr_start;
        for (auto edge_index : lv3_edge_indices) {
            // dist array
            const Addr dist_vaddr =
                dist_array_start_vaddr + edge_index * 4;
            const Addr dist_vaddr_block_aligned = \
                (dist_vaddr >> BLOCK_SHIFT) << BLOCK_SHIFT;
            // We add expected prefetches
            workItem->addExpectedPrefetch(dist_vaddr_block_aligned, 3);
            warnIfOutsideRanges(work_vaddr, dist_vaddr_block_aligned);
            PREFETCHER_TRACE_DEBUG(
                "Work Item = 0x%llx, dist = 0x%llx\n",
                work_vaddr, dist_vaddr_block_aligned
            );
        }
    }

    return workItem;
}

SSSPPrefetchKernel3Generator::SSSPPrefetchKernel3Generator(
    std::string _name,
    const uint64_t _job_id, const uint64_t _core_id,
    const uint64_t _software_hint_distance,
    const uint64_t _prefetch_distance_offset_from_software_hint,
    PrefetcherWorkTracker* _work_tracker
) : PrefetchGenerator(
    _name,
    _job_id, _core_id,
    _software_hint_distance, _prefetch_distance_offset_from_software_hint,
    _work_tracker
    )
{
}

std::shared_ptr<WorkItem>
SSSPPrefetchKernel3Generator::execute_kernel(Addr work_data)
{
    // This kernel does not generate prefetches. Instead, it just updates the
    // SSSP distance threshold in the prefetch context, which is used by other
    // kernels to determine whether to generate prefetches or not.

    // Update the SSSP distance threshold
    prefetch_context->setSSSPCurrentDistanceThreshold(core_id, work_data);

    return nullptr;
}

};  // namespace gem5
