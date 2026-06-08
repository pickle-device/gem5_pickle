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

    const uint64_t idel_total_elems =
        work_tracker->job_descriptor->get_array(0).num_elements();
    const uint64_t num_elements = idel_total_elems / IDEL_ELEMS_PER_IE;

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

    // Level 0: idel(:,:,:,ie)  (10 cache lines of int32)
    {
        const Addr base =
            work_tracker->job_descriptor->get_array(0).vaddr_start;
        const Addr slab_start = base + element_id * IDEL_BYTES_PER_IE;
        const Addr slab_end   = slab_start + IDEL_BYTES_PER_IE;

        for (Addr block = slab_start & BLOCK_MASK;
             block <= ((slab_end - 1) & BLOCK_MASK);
             block += BLOCK_SIZE) {
            workItem->addExpectedPrefetch(block, 0);
            warnIfOutsideRanges(element_id, block);
        }
    }

    // Level 1: tx(:,:,:,ie)   (16 cache lines of double)
    {
        const Addr base =
            work_tracker->job_descriptor->get_array(1).vaddr_start;
        const Addr cube_start = base + element_id * TX_BYTES_PER_IE;
        const Addr cube_end   = cube_start + TX_BYTES_PER_IE;

        for (Addr block = cube_start & BLOCK_MASK;
             block <= ((cube_end - 1) & BLOCK_MASK);
             block += BLOCK_SIZE) {
            workItem->addExpectedPrefetch(block, 1);
            warnIfOutsideRanges(element_id, block);
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
    const CbcMode  _cbc_mode,
    PrefetcherWorkTracker* _work_tracker
) : PrefetchGenerator(
        _name,
        _job_id, _core_id,
        _software_hint_distance, _prefetch_distance_offset_from_software_hint,
        _work_tracker
    ),
    cbc_mode(_cbc_mode)
{
}

std::shared_ptr<WorkItem>
UATransferMortarPrefetchGenerator::execute_kernel(Addr work_data)
{
    using namespace ua_constants;

    const uint64_t element_id = work_data
        + software_hint_distance
        - prefetch_distance_offset_from_software_hint;

    const uint64_t idmo_total_elems =
        work_tracker->job_descriptor->get_array(0).num_elements();
    const uint64_t num_elements = idmo_total_elems / IDMO_ELEMS_PER_IE;

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

    const Addr cbc_base =
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
//   1. Lazily load idmo cache lines only as values are needed.
//   2. Always emit: 4 corner igs, 9 face-interior igs (in (1,1) plane).
//   3. For each of the 4 edges:
//        probe = idmo(corner-test position)
//        if probe != 0 (edge is nonconforming):
//          if nc_edge_emits_work:   emit 10 igs (idmo(j, *, ije1, ije2))
//          else                     /* TransfbC: skip edge */
//        else (edge is conforming):
//          emit 3 igs from the (1,1) plane along that edge
//
// Line caching: at most 8 distinct idmo cache lines could be touched per
// face block (worst-case alignment). We keep a small array-based cache.

namespace {

constexpr int kMaxFaceBlockLines = 8;

struct IdmoLineCache
{
    Addr            vaddrs[kMaxFaceBlockLines];
    const int32_t*  datas [kMaxFaceBlockLines];
    int             size;

    IdmoLineCache() : size(0) {
        for (int i = 0; i < kMaxFaceBlockLines; i++) {
            vaddrs[i] = (Addr)-1;
            datas [i] = nullptr;
        }
    }

    const int32_t* lookup(Addr line_vaddr) const {
        for (int i = 0; i < size; i++) {
            if (vaddrs[i] == line_vaddr) return datas[i];
        }
        return nullptr;
    }

    void insert(Addr line_vaddr, const int32_t* data) {
        if (size >= kMaxFaceBlockLines) return; // shouldn't happen
        vaddrs[size] = line_vaddr;
        datas [size] = data;
        size++;
    }
};

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

} // anonymous namespace

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

    IdmoLineCache cache;
    bool any_failure = false;

    // Load a line once, mark it as an expected prefetch, return data ptr.
    auto load_line = [&](Addr line_vaddr) -> const int32_t* {
        const int32_t* hit = cache.lookup(line_vaddr);
        if (hit != nullptr) return hit;

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
            return nullptr;
        }
        workItem->addExpectedPrefetch(line_vaddr, idmo_level);
        warnIfOutsideRanges(element_id, line_vaddr);
        const int32_t* data = pkt->getConstPtr<int32_t>();
        cache.insert(line_vaddr, data);
        return data;
    };

    // Fetch a single idmo entry (1-based Fortran indices).
    auto get_idmo = [&](int i, int j, int ije1, int ije2) -> int32_t {
        const Addr vaddr      = face_block_base
                              + idmo_flat_offset(i, j, ije1, ije2)
                                * IDX_ITEM_SIZE;
        const Addr line_vaddr = vaddr & BLOCK_MASK;
        const int32_t* data   = load_line(line_vaddr);
        if (data == nullptr) return 0;
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

} // namespace gem5
