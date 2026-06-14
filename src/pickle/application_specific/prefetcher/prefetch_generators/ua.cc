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
idmo_flat_offset(int i, int j, int ije1, int ije2)
{
    using namespace ua_constants;
    return (uint64_t)(i - 1)
         + LX1 * (uint64_t)(j - 1)
         + LX1 * LX1 * (uint64_t)(ije1 - 1)
         + LX1 * LX1 * LNJE * (uint64_t)(ije2 - 1);
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
            lv1_tx_indices.reserve(LX1 * LX1 * NSIDES);

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
//
// Three-mode dispatch:
//
//   Ignore   -> emit entire idmo slab (no cbc, no per-face specialization).
//   Transf   -> read cbc(:, ie); for cbc==3 face emit full block,
//               otherwise dispatch per face/per edge (Transf semantics:
//               NC edge contributes 10 tmor prefetches, conforming edge
//               contributes 3).
//   TransfbC -> read cbc(:, ie); for cbc==3 face skip entirely,
//               otherwise dispatch per face/per edge with TransfbC
//               semantics (NC edge contributes 0 prefetches; only
//               conforming edges do work).
//
// Common helpers (emitFaceFull, emitFaceConforming, readCbcRow) factor out
// the per-face emission so execute_kernel stays a thin dispatcher.

UATransferMortarPrefetchGenerator::UATransferMortarPrefetchGenerator(
    std::string _name,
    const uint64_t _job_id, const uint64_t _core_id,
    const uint64_t _software_hint_distance,
    const uint64_t _prefetch_distance_offset_from_software_hint,
    // transf or transfb or transfb_c or transfb_c_2
    const std::string _function,
    const bool _cbc_optimization_enabled,
    PrefetcherWorkTracker* _work_tracker
) : PrefetchGenerator(
        _name,
        _job_id, _core_id,
        _software_hint_distance, _prefetch_distance_offset_from_software_hint,
        _work_tracker
    )
{
    if (_cbc_optimization_enabled) {
        if (_function == "transfb_c" || _function == "transfb_c_2") {
            cbc_mode = CbcMode::TransfbC;
        } else if (_function == "transf" || _function == "transfb") {
            cbc_mode = CbcMode::Transf;
        } else {
            panic("Invalid function name: %s\n", _function.c_str());
        }
    } else {
        cbc_mode = CbcMode::Ignore;
    }
}

std::shared_ptr<WorkItem>
UATransferMortarPrefetchGenerator::execute_kernel(Addr work_data)
{
    using namespace ua_constants;

    const uint64_t element_id = work_data
        + software_hint_distance
        - prefetch_distance_offset_from_software_hint;

    const uint64_t num_elements = prefetch_context->getUANumElements(core_id);

    PREFETCHER_TRACE_DEBUG(
        "Mortar(mode=%d): work_data=0x%llx element_id=0x%llx "
        "num_elements=0x%llx\n",
        (int) cbc_mode, work_data, element_id, num_elements
    );

    if (element_id >= num_elements) {
        return nullptr;
    }

    std::shared_ptr<WorkItem> workItem(new WorkItem(element_id));

    const Addr idmo_base =
        work_tracker->job_descriptor->get_array(0).vaddr_start;
    const Addr leaf_base =
        work_tracker->job_descriptor->get_array(1).vaddr_start;

    // ---- Ignore mode: emit the entire slab, one face at a time.
    if (cbc_mode == CbcMode::Ignore) {
        for (uint64_t f = 0; f < NSIDES; f++) {
            const Addr face_block_base =
                idmo_base + element_id * IDMO_BYTES_PER_IE
                          + f * IDMO_BYTES_PER_FACE;
            if (!emitFaceFull(element_id, f, face_block_base, leaf_base,
                              workItem,
                              /*idmo_level=*/0, /*leaf_level=*/1))
                return nullptr;
        }
        return workItem;
    }

    // ---- Transf / TransfbC modes: must read cbc(:, ie) first.
    // If the job descriptor does not actually carry cbc (array 2 missing or
    // empty) fall back to Ignore behavior so the run still produces results.
    bool have_cbc = false;
    if (work_tracker->job_descriptor->get_num_arrays() > 2) {
        have_cbc = work_tracker->job_descriptor->get_array(2).num_elements()
                   > 0;
    }
    if (!have_cbc) {
        PREFETCHER_TRACE_DEBUG(
            "cbc array missing; falling back to Ignore-mode emission\n"
        );
        for (uint64_t f = 0; f < NSIDES; f++) {
            const Addr face_block_base =
                idmo_base + element_id * IDMO_BYTES_PER_IE
                          + f * IDMO_BYTES_PER_FACE;
            if (!emitFaceFull(element_id, f, face_block_base, leaf_base,
                              workItem,
                              /*idmo_level=*/0, /*leaf_level=*/1))
                return nullptr;
        }
        return workItem;
    }

    int32_t cbc_row[NSIDES];
    if (!readCbcRow(element_id, cbc_row, workItem, /*cbc_level=*/0)) {
        return nullptr;
    }

    const bool nc_edge_emits_work = (cbc_mode == CbcMode::Transf);

    for (uint64_t f = 0; f < NSIDES; f++) {
        const Addr face_block_base =
            idmo_base + element_id * IDMO_BYTES_PER_IE
                      + f * IDMO_BYTES_PER_FACE;

        const bool is_nc_face = (cbc_row[f] == 3);

        if (is_nc_face) {
            if (cbc_mode == CbcMode::TransfbC) {
                // Application loop skips this face -> no prefetches.
                PREFETCHER_TRACE_DEBUG(
                    "ie=%llu face=%llu cbc=3 skipped (TransfbC)\n",
                    element_id, f
                );
                continue;
            }
            // Transf: full nnje=2 path, emit the full face block.
            if (!emitFaceFull(element_id, f, face_block_base, leaf_base,
                              workItem,
                              /*idmo_level=*/1, /*leaf_level=*/2))
                return nullptr;
        } else {
            // Conforming face. Per-edge dispatch.
            if (!emitFaceConforming(element_id, f, face_block_base, leaf_base,
                                    nc_edge_emits_work, workItem,
                                    /*idmo_level=*/1, /*leaf_level=*/2))
                return nullptr;
        }
    }

    return workItem;
}

// ---------------------------------------------------------------------------
// readCbcRow
// ---------------------------------------------------------------------------
// cbc(:, ie) is 6 int32 = 24 bytes -> 1 cache line in the common case (or 2
// if the row straddles a line boundary). We mark every line we touch with
// addExpectedPrefetch since the application will read them anyway.

bool
UATransferMortarPrefetchGenerator::readCbcRow(
    uint64_t                   element_id,
    int32_t                    cbc_row[ua_constants::NSIDES],
    std::shared_ptr<WorkItem>& workItem,
    uint64_t                   cbc_level)
{
    using namespace ua_constants;
    constexpr Addr BLOCK_SHIFT = 6;
    constexpr Addr BLOCK_SIZE  = 1ULL << BLOCK_SHIFT;
    constexpr Addr BLOCK_MASK  = ~(BLOCK_SIZE - 1);

    const Addr cbc_base   =
        work_tracker->job_descriptor->get_array(2).vaddr_start;
    const Addr row_start  = cbc_base + element_id * CBC_BYTES_PER_IE;
    const Addr row_end    = row_start + CBC_BYTES_PER_IE;
    const Addr first_line = row_start & BLOCK_MASK;
    const Addr last_line  = (row_end - 1) & BLOCK_MASK;

    for (uint64_t f = 0; f < NSIDES; f++) cbc_row[f] = 0;

    for (Addr line = first_line; line <= last_line; line += BLOCK_SIZE) {
        bool success = false;
        PREFETCHER_WORK_TRACKER_DEBUG(
            "Fetching cbc line 0x%llx (ie=%llu)\n", line, element_id
        );
        PacketPtr pkt = work_tracker->owner->zeroCycleLoadWithVAddr(
            line, success);
        if (!success) {
            PREFETCHER_TRACE_DEBUG(
                "Failed to fetch cbc line, ie=%llu vaddr=0x%llx\n",
                element_id, line
            );
            return false;
        }
        workItem->addExpectedPrefetch(line, cbc_level);
        warnIfOutsideRanges(element_id, line);

        const int32_t* data    = pkt->getConstPtr<int32_t>();
        const Addr     line_end = line + BLOCK_SIZE;
        for (uint64_t f = 0; f < NSIDES; f++) {
            const Addr face_vaddr = row_start + f * CBC_ITEM_SIZE;
            if (face_vaddr >= line && face_vaddr < line_end) {
                cbc_row[f] = data[(face_vaddr - line) / CBC_ITEM_SIZE];
            }
        }
    }

    PREFETCHER_TRACE_DEBUG(
        "ie=%llu cbc=[%d %d %d %d %d %d]\n",
        element_id,
        cbc_row[0], cbc_row[1], cbc_row[2],
        cbc_row[3], cbc_row[4], cbc_row[5]
    );
    return true;
}

// ---------------------------------------------------------------------------
// emitFaceFull
// ---------------------------------------------------------------------------
// Walk all 7 (or 8 with worst-case alignment) lines of the per-face idmo
// block, extract every nonzero int32 ig, and emit a tmor prefetch for the
// line containing tmor[ig].

bool
UATransferMortarPrefetchGenerator::emitFaceFull(
    uint64_t                   element_id,
    uint64_t                   face_idx,
    Addr                       face_block_base,
    Addr                       leaf_base,
    std::shared_ptr<WorkItem>& workItem,
    uint64_t                   idmo_level,
    uint64_t                   leaf_level)
{
    using namespace ua_constants;
    constexpr Addr BLOCK_SHIFT = 6;
    constexpr Addr BLOCK_SIZE  = 1ULL << BLOCK_SHIFT;
    constexpr Addr BLOCK_MASK  = ~(BLOCK_SIZE - 1);

    const Addr face_end   = face_block_base + IDMO_BYTES_PER_FACE;
    const Addr first_line = face_block_base & BLOCK_MASK;
    const Addr last_line  = (face_end - 1)  & BLOCK_MASK;

    for (Addr line = first_line; line <= last_line; line += BLOCK_SIZE) {
        bool success = false;
        PREFETCHER_WORK_TRACKER_DEBUG(
            "Fetching idmo line 0x%llx (face=%llu ie=%llu full)\n",
            line, face_idx, element_id
        );
        PacketPtr pkt = work_tracker->owner->zeroCycleLoadWithVAddr(
            line, success);
        if (!success) {
            PREFETCHER_TRACE_DEBUG(
                "Failed to fetch idmo line, ie=%llu vaddr=0x%llx\n",
                element_id, line
            );
            return false;
        }
        workItem->addExpectedPrefetch(line, idmo_level);
        warnIfOutsideRanges(element_id, line);

        const int32_t* data = pkt->getConstPtr<int32_t>();
        const Addr line_end = line + BLOCK_SIZE;
        const Addr scan_lo = (face_block_base > line) ? face_block_base : line;
        const Addr scan_hi = (face_end < line_end) ? face_end : line_end;

        for (Addr v = scan_lo; v < scan_hi; v += IDX_ITEM_SIZE) {
            const int32_t ig = data[(v - line) / IDX_ITEM_SIZE];
            if (ig <= 0) continue;

            const Addr leaf_vaddr = leaf_base + (uint64_t)ig * LEAF_ITEM_SIZE;
            const Addr leaf_block = leaf_vaddr & BLOCK_MASK;
            workItem->addExpectedPrefetch(leaf_block, leaf_level);
            warnIfOutsideRanges(element_id, leaf_block);
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// emitFaceConforming
// ---------------------------------------------------------------------------
// Conforming-face emission with per-edge dispatch. Strategy:
//
//   1. Always emit: 4 corner igs, 9 face-interior igs (in (1,1) plane).
//   2. For each of the 4 edges:
//        probe = idmo(corner-test position)
//        if probe != 0 (edge is nonconforming):
//          if nc_edge_emits_work:   emit 10 igs (idmo(j, *, ije1, ije2))
//          else                     /* TransfbC: skip edge */
//        else (edge is conforming):
//          emit 3 igs from the (1,1) plane along that edge
//
// Every idmo entry read goes through a fresh zeroCycleLoadWithVAddr call
// on its enclosing cache line. Duplicate reads (multiple entries in the
// same line, repeated lookups of the probe vs the surrounding entries)
// just produce duplicate addExpectedPrefetch records — the downstream
// prefetcher dedups, so we don't bother with a local cache.

bool
UATransferMortarPrefetchGenerator::emitFaceConforming(
    uint64_t                   element_id,
    uint64_t                   face_idx,
    Addr                       face_block_base,
    Addr                       leaf_base,
    bool                       nc_edge_emits_work,
    std::shared_ptr<WorkItem>& workItem,
    uint64_t                   idmo_level,
    uint64_t                   leaf_level)
{
    using namespace ua_constants;
    constexpr Addr BLOCK_SHIFT = 6;
    constexpr Addr BLOCK_SIZE  = 1ULL << BLOCK_SHIFT;
    constexpr Addr BLOCK_MASK  = ~(BLOCK_SIZE - 1);

    bool any_failure = false;

    // Fetch a single idmo entry (1-based Fortran indices). Each call issues
    // its own zero-cycle load on the enclosing cache line and records its
    // own expected prefetch. Repeated calls landing on the same line are
    // fine; the downstream prefetcher dedups.
    auto get_idmo = [&](int i, int j, int ije1, int ije2) -> int32_t {
        const Addr vaddr      = face_block_base
                              + idmo_flat_offset(i, j, ije1, ije2)
                                * IDX_ITEM_SIZE;
        const Addr line_vaddr = vaddr & BLOCK_MASK;

        bool success = false;
        PREFETCHER_WORK_TRACKER_DEBUG(
            "Fetching idmo line 0x%llx (face=%llu ie=%llu conf)\n",
            line_vaddr, face_idx, element_id
        );
        PacketPtr pkt = work_tracker->owner->zeroCycleLoadWithVAddr(
            line_vaddr, success);
        if (!success) {
            PREFETCHER_TRACE_DEBUG(
                "Failed to fetch idmo line, ie=%llu vaddr=0x%llx (conf)\n",
                element_id, line_vaddr
            );
            any_failure = true;
            return 0;
        }
        workItem->addExpectedPrefetch(line_vaddr, idmo_level);
        warnIfOutsideRanges(element_id, line_vaddr);
        const int32_t* data = pkt->getConstPtr<int32_t>();
        return data[(vaddr - line_vaddr) / IDX_ITEM_SIZE];
    };

    // Emit one tmor prefetch for the line containing tmor[ig].
    auto emit_tmor = [&](int32_t ig) {
        if (ig <= 0) return;
        const Addr leaf_vaddr = leaf_base + (uint64_t)ig * LEAF_ITEM_SIZE;
        const Addr leaf_block = leaf_vaddr & BLOCK_MASK;
        workItem->addExpectedPrefetch(leaf_block, leaf_level);
        warnIfOutsideRanges(element_id, leaf_block);
        PREFETCHER_TRACE_DEBUG(
            "ie=%llu face=%llu conf ig=%d -> tmor line 0x%llx\n",
            element_id, face_idx, ig, leaf_block
        );
    };

    const int LX1i  = (int) LX1;
    const int LNJEi = (int) LNJE;

    // ---- 4 corners (always) ----
    //   ig1 = idmo(1,   1,   1, 1)    at flat offset 0
    //   ig2 = idmo(LX1, 1,   1, 2)    at flat offset 54
    //   ig3 = idmo(1,   LX1, 2, 1)    at flat offset 45
    //   ig4 = idmo(LX1, LX1, 2, 2)    at flat offset 99
    emit_tmor(get_idmo(1,    1,    1, 1));
    emit_tmor(get_idmo(LX1i, 1,    1, 2));
    emit_tmor(get_idmo(1,    LX1i, 2, 1));
    emit_tmor(get_idmo(LX1i, LX1i, 2, 2));

    // ---- Face interior: 9 entries in (1,1) plane, i,col in 2..LX1-1 ----
    for (int col = 2; col <= LX1i - 1; col++) {
        for (int i = 2; i <= LX1i - 1; i++) {
            emit_tmor(get_idmo(i, col, 1, 1));
        }
    }

    // ---- 4 edges, dispatched on probe values ----
    // Edge 1: probe = idmo(LX1, 1, 1, 1)
    {
        const int32_t probe = get_idmo(LX1i, 1, 1, 1);
        if (probe != 0) {
            // Nonconforming edge.
            if (nc_edge_emits_work) {
                // Transf: idmo(j, 1, 1, ije1) for j=1..LX1, ije1=1..LNJE
                for (int ije1 = 1; ije1 <= LNJEi; ije1++) {
                    for (int j = 1; j <= LX1i; j++) {
                        emit_tmor(get_idmo(j, 1, 1, ije1));
                    }
                }
            }
            // TransfbC: emit nothing for NC edge.
        } else {
            // Conforming edge: idmo(i, 1, 1, 1) for i=2..LX1-1
            for (int i = 2; i <= LX1i - 1; i++) {
                emit_tmor(get_idmo(i, 1, 1, 1));
            }
        }
    }

    // Edge 2: probe = idmo(LX1, 2, 1, 2)
    {
        const int32_t probe = get_idmo(LX1i, 2, 1, 2);
        if (probe != 0) {
            if (nc_edge_emits_work) {
                // Transf: idmo(LX1, j, ije1, 2) for j=1..LX1, ije1=1..LNJE
                for (int ije1 = 1; ije1 <= LNJEi; ije1++) {
                    for (int j = 1; j <= LX1i; j++) {
                        emit_tmor(get_idmo(LX1i, j, ije1, 2));
                    }
                }
            }
        } else {
            // Conforming edge: idmo(LX1, i, 1, 1) for i=2..LX1-1
            for (int i = 2; i <= LX1i - 1; i++) {
                emit_tmor(get_idmo(LX1i, i, 1, 1));
            }
        }
    }

    // Edge 3: probe = idmo(2, LX1, 2, 1)
    {
        const int32_t probe = get_idmo(2, LX1i, 2, 1);
        if (probe != 0) {
            if (nc_edge_emits_work) {
                // Transf: idmo(j, LX1, 2, ije1) for j=1..LX1, ije1=1..LNJE
                for (int ije1 = 1; ije1 <= LNJEi; ije1++) {
                    for (int j = 1; j <= LX1i; j++) {
                        emit_tmor(get_idmo(j, LX1i, 2, ije1));
                    }
                }
            }
        } else {
            // Conforming edge: idmo(i, LX1, 1, 1) for i=2..LX1-1
            for (int i = 2; i <= LX1i - 1; i++) {
                emit_tmor(get_idmo(i, LX1i, 1, 1));
            }
        }
    }

    // Edge 4: probe = idmo(1, LX1, 1, 1)
    {
        const int32_t probe = get_idmo(1, LX1i, 1, 1);
        if (probe != 0) {
            if (nc_edge_emits_work) {
                // Transf: idmo(1, j, ije1, 1) for j=1..LX1, ije1=1..LNJE
                for (int ije1 = 1; ije1 <= LNJEi; ije1++) {
                    for (int j = 1; j <= LX1i; j++) {
                        emit_tmor(get_idmo(1, j, ije1, 1));
                    }
                }
            }
        } else {
            // Conforming edge: idmo(1, i, 1, 1) for i=2..LX1-1
            for (int i = 2; i <= LX1i - 1; i++) {
                emit_tmor(get_idmo(1, i, 1, 1));
            }
        }
    }

    return !any_failure;
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
