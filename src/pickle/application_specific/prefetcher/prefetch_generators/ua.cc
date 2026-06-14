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

    const uint64_t num_elements = prefetch_context->getUANumElements(core_id);

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

std::shared_ptr<WorkItem>
UATransferMortarPrefetchGenerator::execute_kernel(Addr work_data)
{
    using namespace ua_constants;

    const uint64_t element_id = work_data
        + software_hint_distance
        - prefetch_distance_offset_from_software_hint;
    const Addr work_item = element_id;

    const uint64_t num_elements = prefetch_context->getUANumElements(core_id);

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

    std::vector<uint64_t> lv1_indices;
    lv1_indices.reserve(LX1 * LX1 * LNJE * LNJE * NSIDES);

    if (cbc_optimization_enabled) {
        panic("CBC optimization not implemented yet\n");
    } else { // prefetch everything
        // Level 0: idmo(:,:,:,ie)
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
    prefetch_context->setUANumElements(core_id, work_data);
    PREFETCHER_TRACE_DEBUG(
        "UANumElementsUpdateKernel::execute_kernel "
        "core_id=0x%llx work_data=0x%llx\n",
        core_id, work_data
    );
    return nullptr;
}

} // namespace gem5
