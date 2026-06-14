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

#include "pickle/application_specific/prefetcher/prefetch_generators/ua.hh"

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/PickleDevicePrefetcherTrace.hh"
#include "debug/PickleDevicePrefetcherWorkTrackerDebug.hh"
#include "mem/packet.hh"
#include "pickle/application_specific/prefetcher/pickle_prefetcher.hh"
#include "pickle/application_specific/prefetcher/prefetcher_work_tracker.hh"

namespace gem5
{


namespace {

// Fortran column-major flat offset within a single (iface, ie) block:
//   idmo(i, j, ije1, ije2)  with 1-based indices in [1..LX1] x [1..LX1]
//                                                 x [1..LNJE] x [1..LNJE]
static inline uint64_t
idmo_flat_index(int i, int j, int ije1, int ije2, uint64_t iface, uint64_t ie)
{
    using namespace ua_constants;
    return (uint64_t)(i - 1)
         + LX1 * (uint64_t)(j - 1)
         + LX1 * LX1 * (uint64_t)(ije1 - 1)
         + LX1 * LX1 * LNJE * (uint64_t)(ije2 - 1)
         + LX1 * LX1 * LNJE * LNJE * (uint64_t)(iface - 1)
         + LX1 * LX1 * LNJE * LNJE * NSIDES * (uint64_t)(ie - 1);
}

static inline uint64_t
idel_flat_index(uint64_t i, uint64_t j, uint64_t iface, uint64_t ie)
{
    using namespace ua_constants;
    return (uint64_t)(i - 1)
         + LX1 * (uint64_t)(j - 1)
         + LX1 * LX1 * (uint64_t)(iface - 1)
         + LX1 * LX1 * NSIDES * (uint64_t)(ie - 1);
}

static inline uint64_t
cbc_flat_index(uint64_t iface, uint64_t ie)
{
    using namespace ua_constants;
    return (uint64_t)(iface - 1) + NSIDES * (uint64_t)(ie - 1);
}

static inline uint64_t
get_32bit_data(
    const Addr vaddr, bool &success, PicklePrefetcher* owner
) {
    const Addr BLOCK_SHIFT = 6;
    const Addr BLOCK_SIZE  = 1ULL << BLOCK_SHIFT;
    Addr vaddr_block_aligned = (vaddr >> BLOCK_SHIFT) << BLOCK_SHIFT;
    success = false;
    PacketPtr pkt = owner->zeroCycleLoadWithVAddr(
        vaddr_block_aligned, success
    );
    if (!success) {
        return 0;
    }
    const Addr index = (vaddr - vaddr_block_aligned) / 4;
    return pkt->getPtr<uint32_t>()[index];
}

static inline uint64_t
get_64bit_data(
    const Addr vaddr, bool &success, PicklePrefetcher* owner
) {
    const Addr BLOCK_SHIFT = 6;
    const Addr BLOCK_SIZE  = 1ULL << BLOCK_SHIFT;
    Addr vaddr_block_aligned = (vaddr >> BLOCK_SHIFT) << BLOCK_SHIFT;
    success = false;
    PacketPtr pkt = owner->zeroCycleLoadWithVAddr(
        vaddr_block_aligned, success
    );
    if (!success) {
        return 0;
    }
    const Addr index = (vaddr - vaddr_block_aligned) / 8;
    return pkt->getPtr<uint64_t>()[index];
}

} // anonymous namespace

// ===========================================================================
// UATransferDensePrefetchGenerator
// ===========================================================================

UATransferDensePrefetchGenerator::UATransferDensePrefetchGenerator(
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
UATransferDensePrefetchGenerator::execute_kernel(Addr work_data)
{
    using namespace ua_constants;

    const uint64_t element_id = work_data
        + software_hint_distance
        - prefetch_distance_offset_from_software_hint;
    const Addr work_item = element_id;

    const uint64_t num_elements = prefetch_context->getUANumElements();

    PREFETCHER_TRACE_DEBUG(
        "Dense: work_data=0x%llx element_id=0x%llx num_elements=0x%llx\n",
        work_data, element_id, num_elements
    );

    if (element_id >= num_elements) {
        return nullptr;
    }

    std::shared_ptr<WorkItem> workItem(new WorkItem(element_id));

    constexpr Addr BLOCK_SHIFT = 6;
    constexpr Addr BLOCK_SIZE  = 1ULL << BLOCK_SHIFT;
    constexpr Addr BLOCK_MASK  = ~(BLOCK_SIZE - 1);

    std::vector<uint64_t> lv1_tx_indices;
    lv1_tx_indices.reserve(LX1 * LX1 * NSIDES);

    // Level 0: idel(:,:,:,ie)
    {
        const Addr idel_base =
            work_tracker->job_descriptor->get_array(0).vaddr_start;
        const Addr idel_start =
            idel_base + idel_flat_index(1, 1, 1, element_id) * IDX_ITEM_SIZE;
        const Addr idel_end =
            idel_base + idel_flat_index(LX1, LX1, NSIDES, element_id)
                * IDX_ITEM_SIZE;
        for (
            Addr index_addr = idel_start;
            index_addr < idel_end;
            index_addr += IDX_ITEM_SIZE
        )
        {
            Addr curr_block_vaddr = 1;
            PacketPtr pkt = nullptr;
            uint32_t* data_ptr = nullptr;

            Addr index_vaddr_block_aligned = \
                (index_addr >> BLOCK_SHIFT) << BLOCK_SHIFT;
            if (index_vaddr_block_aligned != curr_block_vaddr) {
                bool success = false;
                DPRINTF(
                    PickleDevicePrefetcherWorkTrackerDebug,
                    "Fetching lv0 vaddr 0x%llx\n",
                    index_vaddr_block_aligned
                );
                pkt = work_tracker->owner->zeroCycleLoadWithVAddr(
                    index_vaddr_block_aligned, success
                );
                if (!success) {
                    DPRINTF(
                        PickleDevicePrefetcherTrace,
                        "Failed to fetch level = 0, Work Item = 0x%llx, "
                        "vaddr = 0x%llx\n",
                        work_item, index_vaddr_block_aligned
                    );
                    return nullptr;
                }
                curr_block_vaddr = index_vaddr_block_aligned;
                data_ptr = pkt->getPtr<uint32_t>();
                // We add expected prefetches
                workItem->addExpectedPrefetch(curr_block_vaddr, 0);
                warnIfOutsideRanges(element_id, curr_block_vaddr);
            }
            const Addr tx_index =
                (index_addr - curr_block_vaddr) / IDX_ITEM_SIZE;
            // Fortran indices are 1-based indexed, but for addresses, we need
            // 0-based indices. So, subtract 1 to get the 0-based index
            lv1_tx_indices.push_back(data_ptr[tx_index] - 1);
            DPRINTF(
                PickleDevicePrefetcherTrace,
                "Work Item = 0x%llx, lv1_tx_index = %lld\n",
                work_item, lv1_tx_indices.back()
            );
        }
    }

    // Level 1: tx(idel(:,:,:,ie))
    {
        const Addr tx_base =
            work_tracker->job_descriptor->get_array(1).vaddr_start;
        for (auto const& tx_index : lv1_tx_indices) {
            const Addr tx_vaddr =
                tx_base + tx_index * LEAF_ITEM_SIZE;
            const Addr tx_vaddr_block_aligned =
                (tx_vaddr >> BLOCK_SHIFT) << BLOCK_SHIFT;
            workItem->addExpectedPrefetch(tx_vaddr_block_aligned, 1);
            warnIfOutsideRanges(element_id, tx_vaddr_block_aligned);
            DPRINTF(
                PickleDevicePrefetcherTrace,
                "Work Item = 0x%llx, tx_addr = 0x%llx\n",
                work_item, tx_vaddr_block_aligned
            );
        }
    }

    return workItem;
}

// ===========================================================================
// UATransferMortarPrefetchGenerator
// ===========================================================================

UATransferMortarPrefetchGenerator::UATransferMortarPrefetchGenerator(
    std::string _name,
    const uint64_t _job_id, const uint64_t _core_id,
    const uint64_t _software_hint_distance,
    const uint64_t _prefetch_distance_offset_from_software_hint,
    const bool _cbc_optimization_enabled,
    PrefetcherWorkTracker* _work_tracker
) : PrefetchGenerator(
        _name,
        _job_id, _core_id,
        _software_hint_distance, _prefetch_distance_offset_from_software_hint,
        _work_tracker
    ),
    cbc_optimization_enabled(_cbc_optimization_enabled)
{
}

void
UATransferMortarPrefetchGenerator::add_prefetch(
    const Addr vaddr,
    const uint64_t prefetch_level,
    const uint64_t element_id,
    std::shared_ptr<WorkItem> work_item
)
{
    DPRINTF(
        PickleDevicePrefetcherWorkTrackerDebug,
        "Fetching lv%llu vaddr 0x%llx\n",
        prefetch_level, vaddr
    );
    constexpr Addr BLOCK_SHIFT = 6;
    const Addr vaddr_block_aligned = (vaddr >> BLOCK_SHIFT) << BLOCK_SHIFT;
    work_item->addExpectedPrefetch(vaddr_block_aligned, prefetch_level);
    warnIfOutsideRanges(element_id, vaddr_block_aligned);
    DPRINTF(
        PickleDevicePrefetcherWorkTrackerDebug,
        "Fetching lv%llu vaddr 0x%llx\n",
        prefetch_level, vaddr_block_aligned
    );
}

std::shared_ptr<WorkItem>
UATransferMortarPrefetchGenerator::execute_kernel(Addr work_data)
{
    using namespace ua_constants;

    const uint64_t element_id = work_data
        + software_hint_distance
        - prefetch_distance_offset_from_software_hint;
    const Addr work_item = element_id;

    const uint64_t num_elements = prefetch_context->getUANumElements();

    PREFETCHER_TRACE_DEBUG(
        "Dense: work_data=0x%llx element_id=0x%llx num_elements=0x%llx\n",
        work_data, element_id, num_elements
    );

    if (element_id >= num_elements) {
        return nullptr;
    }

    std::shared_ptr<WorkItem> workItem(new WorkItem(element_id));

    constexpr Addr BLOCK_SHIFT = 6;
    constexpr Addr BLOCK_SIZE  = 1ULL << BLOCK_SHIFT;
    constexpr Addr BLOCK_MASK  = ~(BLOCK_SIZE - 1);

    std::vector<Addr> lv1_addresses;
    std::vector<uint64_t> lv1_indices;
    std::vector<Addr> lv2_addresses;
    std::vector<uint64_t> lv2_indices;
    lv1_addresses.reserve(LX1 * LX1 * LNJE * LNJE * NSIDES);
    lv1_indices.reserve(LX1 * LX1 * LNJE * LNJE * NSIDES);
    lv2_addresses.reserve(LX1 * LX1 * LNJE * LNJE * NSIDES);
    lv2_indices.reserve(LX1 * LX1 * LNJE * LNJE * NSIDES);
    bool is_edge_1_conforming = false;
    bool is_edge_2_conforming = false;
    bool is_edge_3_conforming = false;
    bool is_edge_4_conforming = false;
    bool success = false;

    if (cbc_optimization_enabled) {
        const Addr idmo_base =
            work_tracker->job_descriptor->get_array(0).vaddr_start;
        const Addr pmorx_base =
            work_tracker->job_descriptor->get_array(1).vaddr_start;
        const Addr cbc_base =
            work_tracker->job_descriptor->get_array(2).vaddr_start;
        bool is_conforming = false;
        // iterate through faces, each can be conforming/non-conforming
        for (uint64_t iface = 1; iface <= NSIDES; iface++) {
            // level 1: load the cbc(iface, ie) value
            const Addr cbc_vaddr =
                cbc_base + cbc_flat_index(iface, element_id) * CBC_ITEM_SIZE;
            const Addr cbc_vaddr_block_aligned =
                (cbc_vaddr >> BLOCK_SHIFT) << BLOCK_SHIFT;
            workItem->addExpectedPrefetch(cbc_vaddr_block_aligned, 0);
            warnIfOutsideRanges(element_id, cbc_vaddr_block_aligned);
            DPRINTF(
                PickleDevicePrefetcherWorkTrackerDebug,
                "Fetching lv0 vaddr 0x%llx\n",
                cbc_vaddr_block_aligned
            );
            uint64_t cbc_val = get_32bit_data(
                cbc_vaddr, success, work_tracker->owner
            );
            if (!success) {
                DPRINTF(
                    PickleDevicePrefetcherTrace,
                    "Failed to fetch level = 0, vaddr = 0x%llx\n",
                    cbc_vaddr_block_aligned
                );
               continue;
            }
            is_conforming = (cbc_val == 3);
            DPRINTF(
                PickleDevicePrefetcherWorkTrackerDebug,
                "cbc_val = 0x%llx is_conforming = %d\n",
                cbc_val, is_conforming
            );
            // level 2: load the indices from idmo
            if (!is_conforming) {
                for (uint64_t ije1=1; ije1 <= LNJE; ije1++) {
                    for (uint64_t ije2=1; ije2 <= LNJE; ije2++) {
                        for (uint64_t col=1; col <= LX1; col++) {
                            // idmo(i,col,ije1,ije2,iface,ie)
                            const Addr col_idx_vaddr =
                                idmo_base + idmo_flat_index(
                                    1, 1, ije1, ije2, iface, element_id
                                ) * IDX_ITEM_SIZE;
                            lv1_addresses.push_back(col_idx_vaddr);
                            // ------------------------
                            for (uint64_t i = 2; i <= LX1-1; i++) {
                                for (uint64_t j = 1; j <= LX1; j++) {
                                    // idmo(j,col,ije1,ije2,iface,ie)
                                    const Addr idx_vaddr =
                                        idmo_base + idmo_flat_index(
                                            j, col, ije1, ije2, iface,
                                            element_id
                                        ) * IDX_ITEM_SIZE;
                                    lv1_addresses.push_back(idx_vaddr);
                                    // ------------------------
                                } // for j
                            } // for i
                        } // for col
                    } // for ije2
                } // for ije1
            } else { // if conforming
                // face interior
                for (uint64_t col = 2; col <= LX1-1; col++) {
                    for (uint64_t i = 2; i <= LX1-1; i++) {
                        // idmo(i,col,1,1,iface,ie)
                        const Addr idx_vaddr =
                            idmo_base + idmo_flat_index(
                                i, col, 1, 1, iface, element_id
                            ) * IDX_ITEM_SIZE;
                        lv1_addresses.push_back(idx_vaddr);
                        // ------------------------
                    } // for i
                } // for col

{
                // check if edge 1 is conforming
                // idmo(lx1,1,1,1,iface,ie)
                const Addr edge_1_vaddr = idmo_base + idmo_flat_index(
                    LX1, 1, 1, 1, iface, element_id
                ) * IDX_ITEM_SIZE;
                const Addr edge_1_vaddr_block_aligned =
                    (edge_1_vaddr >> BLOCK_SHIFT) << BLOCK_SHIFT;
                workItem->addExpectedPrefetch(edge_1_vaddr_block_aligned, 1);
                warnIfOutsideRanges(element_id, edge_1_vaddr_block_aligned);
                DPRINTF(
                    PickleDevicePrefetcherWorkTrackerDebug,
                    "Fetching lv1 vaddr 0x%llx\n",
                    edge_1_vaddr_block_aligned
                );
                uint64_t edge_1_val = get_32bit_data(
                    edge_1_vaddr, success, work_tracker->owner
                );
                if (!success) {
                    DPRINTF(
                        PickleDevicePrefetcherTrace,
                        "Failed to fetch level = 1, vaddr = 0x%llx\n",
                        edge_1_vaddr_block_aligned
                    );
                    continue;
                }
                is_edge_1_conforming = (edge_1_val == 0);
                DPRINTF(
                    PickleDevicePrefetcherWorkTrackerDebug,
                    "edge_1_val = 0x%llx is_edge_1_conforming = %d\n",
                    edge_1_val, is_edge_1_conforming
                );
                if (!is_edge_1_conforming) {
                    for (uint64_t i = 2; i <= LX1-1; i++) {
                        for (uint64_t ije1 = 1; ije1 <= 2; ije1++) {
                            for (uint64_t j = 1; j <= LX1; j++) {
                                // idmo(j,1,1,ije1,iface,ie)
                                const Addr idx_vaddr =
                                    idmo_base + idmo_flat_index(
                                        j, 1, 1, ije1, iface, element_id
                                    ) * IDX_ITEM_SIZE;
                                lv2_addresses.push_back(idx_vaddr);
                                // ------------------------
                            } // for j
                        } // for ije1
                    } // for i
                } else { // edge 1 is conforming
                    for (uint64_t i = 2; i <= LX1-1; i++) {
                        // idmo(i,1,1,1,iface,ie)
                        const Addr idx_vaddr =
                            idmo_base + idmo_flat_index(
                                i, 1, 1, 1, iface, element_id
                            ) * IDX_ITEM_SIZE;
                        lv2_addresses.push_back(idx_vaddr);
                        // ------------------------
                    } // for i
                }
}

{
                // check if edge 2 is conforming
                // idmo(lx1,2,1,2,iface,ie)
                const Addr edge_2_vaddr = idmo_base + idmo_flat_index(
                    LX1, 2, 1, 2, iface, element_id
                ) * IDX_ITEM_SIZE;
                const Addr edge_2_vaddr_block_aligned =
                    (edge_2_vaddr >> BLOCK_SHIFT) << BLOCK_SHIFT;
                workItem->addExpectedPrefetch(edge_2_vaddr_block_aligned, 1);
                warnIfOutsideRanges(element_id, edge_2_vaddr_block_aligned);
                DPRINTF(
                    PickleDevicePrefetcherWorkTrackerDebug,
                    "Fetching lv1 vaddr 0x%llx\n",
                    edge_2_vaddr_block_aligned
                );
                uint64_t edge_2_val = get_32bit_data(
                    edge_2_vaddr, success, work_tracker->owner
                );
                if (!success) {
                    DPRINTF(
                        PickleDevicePrefetcherTrace,
                        "Failed to fetch level = 1, vaddr = 0x%llx\n",
                        edge_2_vaddr_block_aligned
                    );
                    continue;
                }
                is_edge_2_conforming = (edge_2_val == 0);
                DPRINTF(
                    PickleDevicePrefetcherWorkTrackerDebug,
                    "edge_2_val = 0x%llx is_edge_2_conforming = %d\n",
                    edge_2_val, is_edge_2_conforming
                );
                if (!is_edge_2_conforming) {
                    for (uint64_t i = 2; i <= LX1-1; i++) {
                        for (uint64_t ije1 = 1; ije1 <= 2; ije1++) {
                            for (uint64_t j = 1; j <= LX1; j++) {
                                // idmo(lx1,j,ije1,2,iface,ie)
                                const Addr idx_vaddr =
                                    idmo_base + idmo_flat_index(
                                        LX1, j, ije1, 2, iface, element_id
                                    ) * IDX_ITEM_SIZE;
                                lv2_addresses.push_back(idx_vaddr);
                                // ------------------------
                            } // for j
                        } // for ije1
                    } // for i
                } else { // edge 2 is conforming
                    for (uint64_t i = 2; i <= LX1-1; i++) {
                        // idmo(lx1,i,1,2,iface,ie)
                        const Addr idx_vaddr =
                            idmo_base + idmo_flat_index(
                                LX1, i, 1, 2, iface, element_id
                            ) * IDX_ITEM_SIZE;
                        lv2_addresses.push_back(idx_vaddr);
                        // ------------------------
                    } // for i
                }
}

{
                // check if edge 3 is conforming
                // idmo(2,lx1,2,1,iface,ie)
                const Addr edge_3_vaddr = idmo_base + idmo_flat_index(
                    2, LX1, 2, 1, iface, element_id
                ) * IDX_ITEM_SIZE;
                const Addr edge_3_vaddr_block_aligned =
                    (edge_3_vaddr >> BLOCK_SHIFT) << BLOCK_SHIFT;
                workItem->addExpectedPrefetch(edge_3_vaddr_block_aligned, 1);
                warnIfOutsideRanges(element_id, edge_3_vaddr_block_aligned);
                DPRINTF(
                    PickleDevicePrefetcherWorkTrackerDebug,
                    "Fetching lv1 vaddr 0x%llx\n",
                    edge_3_vaddr_block_aligned
                );
                uint64_t edge_3_val = get_32bit_data(
                    edge_3_vaddr, success, work_tracker->owner
                );
                if (!success) {
                    DPRINTF(
                        PickleDevicePrefetcherTrace,
                        "Failed to fetch level = 1, vaddr = 0x%llx\n",
                        edge_3_vaddr_block_aligned
                    );
                    continue;
                }
                is_edge_3_conforming = (edge_3_val == 0);
                DPRINTF(
                    PickleDevicePrefetcherWorkTrackerDebug,
                    "edge_3_val = 0x%llx is_edge_3_conforming = %d\n",
                    edge_3_val, is_edge_3_conforming
                );
                if (!is_edge_3_conforming) {
                    for (uint64_t i = 2; i <= LX1-1; i++) {
                        for (uint64_t ije1 = 1; ije1 <= 2; ije1++) {
                            for (uint64_t j = 1; j <= LX1; j++) {
                                // idmo(j,lx1,2,ije1,iface,ie)
                                const Addr idx_vaddr =
                                    idmo_base + idmo_flat_index(
                                        j, LX1, 2, ije1, iface, element_id
                                    ) * IDX_ITEM_SIZE;
                                lv2_addresses.push_back(idx_vaddr);
                                // ------------------------
                            } // for j
                        } // for ije1
                    } // for i
                } else { // edge 3 is conforming
                    for (uint64_t i = 2; i <= LX1-1; i++) {
                        // idmo(i,lx1,2,1,iface,ie)
                        const Addr idx_vaddr =
                            idmo_base + idmo_flat_index(
                                i, LX1, 2, 1, iface, element_id
                            ) * IDX_ITEM_SIZE;
                        lv2_addresses.push_back(idx_vaddr);
                        // ------------------------
                    } // for i
                }
}

{
                // check if edge 4 is conforming
                // idmo(1,lx1,1,1,iface,ie)
                const Addr edge_4_vaddr = idmo_base + idmo_flat_index(
                    1, LX1, 1, 1, iface, element_id
                ) * IDX_ITEM_SIZE;
                const Addr edge_4_vaddr_block_aligned =
                    (edge_4_vaddr >> BLOCK_SHIFT) << BLOCK_SHIFT;
                workItem->addExpectedPrefetch(edge_4_vaddr_block_aligned, 1);
                warnIfOutsideRanges(element_id, edge_4_vaddr_block_aligned);
                DPRINTF(
                    PickleDevicePrefetcherWorkTrackerDebug,
                    "Fetching lv1 vaddr 0x%llx\n",
                    edge_4_vaddr_block_aligned
                );
                uint64_t edge_4_val = get_32bit_data(
                    edge_4_vaddr, success, work_tracker->owner
                );
                if (!success) {
                    DPRINTF(
                        PickleDevicePrefetcherTrace,
                        "Failed to fetch level = 1, vaddr = 0x%llx\n",
                        edge_4_vaddr_block_aligned
                    );
                    continue;
                }
                is_edge_4_conforming = (edge_4_val == 0);
                DPRINTF(
                    PickleDevicePrefetcherWorkTrackerDebug,
                    "edge_4_val = 0x%llx is_edge_4_conforming = %d\n",
                    edge_4_val, is_edge_4_conforming
                );
                if (!is_edge_4_conforming) {
                    for (uint64_t i = 2; i <= LX1-1; i++) {
                        for (uint64_t ije1 = 1; ije1 <= 2; ije1++) {
                            for (uint64_t j = 1; j <= LX1; j++) {
                                // idmo(1,j,ije1,1,iface,ie)
                                const Addr idx_vaddr =
                                    idmo_base + idmo_flat_index(
                                        1, j, ije1, 1, iface, element_id
                                    ) * IDX_ITEM_SIZE;
                                lv2_addresses.push_back(idx_vaddr);
                                // ------------------------
                            } // for j
                        } // for ije1
                    } // for i
                } else { // edge 4 is conforming
                    for (uint64_t i = 2; i <= LX1-1; i++) {
                        // idmo(1,i,1,1,iface,ie)
                        const Addr idx_vaddr =
                            idmo_base + idmo_flat_index(
                                1, i, 1, 1, iface, element_id
                            ) * IDX_ITEM_SIZE;
                        lv2_addresses.push_back(idx_vaddr);
                        // ------------------------
                    } // for i
                }
}
            }
            // now we populate lv1 addresses to lv1_indices
            for (const auto& lv1_address: lv1_addresses) {
                add_prefetch(lv1_address, 1, element_id, workItem);
                uint64_t lv1_val = get_32bit_data(
                    lv1_address, success, work_tracker->owner
                );
                if (!success) {
                    DPRINTF(
                        PickleDevicePrefetcherTrace,
                        "Failed to fetch level = 1, vaddr = 0x%llx\n",
                        lv1_address
                    );
                    continue;
                }
                lv1_indices.push_back(lv1_val);
            }
            // now we populate lv2 addresses to lv2_indices
            for (const auto& lv2_address: lv2_addresses) {
                add_prefetch(lv2_address, 2, element_id, workItem);
                uint64_t lv2_val = get_32bit_data(
                    lv2_address, success, work_tracker->owner
                );
                if (!success) {
                    DPRINTF(
                        PickleDevicePrefetcherTrace,
                        "Failed to fetch level = 2, vaddr = 0x%llx\n",
                        lv2_address
                    );
                    continue;
                }
                lv2_indices.push_back(lv2_val);
            }
            // now we populate lv3 via pmorx(idmo(a,b,c,d,iface,ie))
            const Addr next_level = lv2_indices.empty() ? 2 : 3;
            for (const auto& lv1_idx_val : lv1_indices) {
                Addr vaddr =
                    pmorx_base + ((lv1_idx_val - 1) * LEAF_ITEM_SIZE);
                add_prefetch(vaddr, next_level, element_id, workItem);
            }
            for (const auto& lv2_idx_val : lv2_indices) {
                Addr vaddr =
                    pmorx_base + ((lv2_idx_val - 1) * LEAF_ITEM_SIZE);
                add_prefetch(vaddr, next_level, element_id, workItem);
            }
        }
    } else { // prefetch everything
        // Level 0: idmo(:,:,:,:,:,ie)
        {
            const Addr idmo_base =
                work_tracker->job_descriptor->get_array(0).vaddr_start;
            const Addr idmo_start =
                idmo_base + idmo_flat_index(
                    1, 1, 1, 1, 1, element_id
                ) * IDX_ITEM_SIZE;
            const Addr idmo_end =
                idmo_base + idmo_flat_index(
                    LX1, LX1, LNJE, LNJE, NSIDES, element_id
                ) * IDX_ITEM_SIZE;
            for (
                Addr index_addr = idmo_start;
                index_addr < idmo_end;
                index_addr += IDX_ITEM_SIZE
            )
            {
                Addr curr_block_vaddr = 1;
                PacketPtr pkt = nullptr;
                uint32_t* data_ptr = nullptr;

                Addr index_vaddr_block_aligned = \
                    (index_addr >> BLOCK_SHIFT) << BLOCK_SHIFT;
                if (index_vaddr_block_aligned != curr_block_vaddr) {
                    bool success = false;
                    DPRINTF(
                        PickleDevicePrefetcherWorkTrackerDebug,
                        "Fetching lv0 vaddr 0x%llx\n",
                        index_vaddr_block_aligned
                    );
                    pkt = work_tracker->owner->zeroCycleLoadWithVAddr(
                        index_vaddr_block_aligned, success
                    );
                    if (!success) {
                        DPRINTF(
                            PickleDevicePrefetcherTrace,
                            "Failed to fetch level = 0, Work Item = 0x%llx, "
                            "vaddr = 0x%llx\n",
                            work_item, index_vaddr_block_aligned
                        );
                        return nullptr;
                    }
                    curr_block_vaddr = index_vaddr_block_aligned;
                    data_ptr = pkt->getPtr<uint32_t>();
                    // We add expected prefetches
                    workItem->addExpectedPrefetch(curr_block_vaddr, 0);
                    warnIfOutsideRanges(element_id, curr_block_vaddr);
                }
                const Addr pmorx_index =
                    (index_addr - curr_block_vaddr) / IDX_ITEM_SIZE;
                // Fortran indices are 1-based indexed, but for addresses,
                // we need 0-based indices. So, subtract 1 to get the
                // 0-based index
                lv1_indices.push_back(data_ptr[pmorx_index] - 1);
                DPRINTF(
                    PickleDevicePrefetcherTrace,
                    "Work Item = 0x%llx, lv1_pmorx_index = %lld\n",
                    work_item, lv1_indices.back()
                );
            }
        }

        // Level 1: pmorx(idel(:,:,:,ie))
        {
            const Addr pmorx_base =
                work_tracker->job_descriptor->get_array(1).vaddr_start;
            for (auto const& pmorx_index : lv1_indices) {
                const Addr pmorx_vaddr =
                    pmorx_base + pmorx_index * LEAF_ITEM_SIZE;
                const Addr pmorx_vaddr_block_aligned =
                    (pmorx_vaddr >> BLOCK_SHIFT) << BLOCK_SHIFT;
                workItem->addExpectedPrefetch(pmorx_vaddr_block_aligned, 1);
                warnIfOutsideRanges(element_id, pmorx_vaddr_block_aligned);
                DPRINTF(
                    PickleDevicePrefetcherTrace,
                    "Work Item = 0x%llx, pmorx_addr = 0x%llx\n",
                    work_item, pmorx_vaddr_block_aligned
                );
            }
        }
    }
    return workItem;
}

UANumElementsUpdateKernel::UANumElementsUpdateKernel(
    std::string _name,
    const uint64_t _job_id, const uint64_t _core_id,
    const uint64_t _software_hint_distance,
    const uint64_t _prefetch_distance_offset_from_software_hint,
    PrefetcherWorkTracker* _work_tracker
) :
    PrefetchGenerator(
        _name, _job_id, _core_id,
        _software_hint_distance, _prefetch_distance_offset_from_software_hint,
        _work_tracker
    )
{
}

std::shared_ptr<WorkItem>
UANumElementsUpdateKernel::execute_kernel(Addr work_data)
{
    prefetch_context->setUANumElements(work_data);
    PREFETCHER_TRACE_DEBUG(
        "UANumElementsUpdateKernel::execute_kernel "
        "work_data=0x%llx\n",
        work_data
    );
    return nullptr;
}

} // namespace gem5
