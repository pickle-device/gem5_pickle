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

#ifndef __UA_PREFETCH_GENERATOR_HH__
#define __UA_PREFETCH_GENERATOR_HH__

#include <cstdint>
#include <memory>
#include <string>

#include "base/logging.hh"
#include "debug/PickleDevicePrefetcherTrace.hh"
#include "debug/PickleDevicePrefetcherWorkTrackerDebug.hh"
#include "pickle/application_specific/prefetcher/prefetch_generators/prefetch_generator.hh"

// ---------------------------------------------------------------------------
// NAS UA Pickle prefetch generators
// ---------------------------------------------------------------------------
//
// Two classes, mapped to four kernel slots emitted by pickle_ua_glue.cc:
//
//   "ua_transf_tx"    -> UATransferDensePrefetchGenerator  (slot 0)
//   "ua_transf_tmor"  -> UATransferMortarPrefetchGenerator (slot 1, transf)
//   "ua_transfb_tx"   -> UATransferDensePrefetchGenerator  (slot 2)
//   "ua_transfb_tmor" -> UATransferMortarPrefetchGenerator (slot 3, transfb)
//
// conforming face branch optimization:
//   * False   — no cbc reads, no per-face specialization. Emit prefetches
//                for every nonzero ig in the entire idmo slab. Use when the
//                glue did not push the cbc array, or when measuring the
//                "uninformed" baseline.
//
//   * True    — for transf / transfb (slots 1, 3). Read cbc(:, ie).
//                For each face f:
//                   cbc(f, ie) == 3 (nonconforming face):
//                     emit prefetches for the whole face block
//                     (matches the nnje=2 path in the application).
//                   cbc(f, ie) != 3 (conforming face):
//                     emit 4 corners + 9 face-interior tmor prefetches;
//                     probe the 4 edge-test positions; per edge emit either
//                     10 prefetches (NC edge) or 3 prefetches (conf edge)
//
// (See README for arithmetic; LX1=5, LNJE=2, NSIDES=6.)
// ---------------------------------------------------------------------------

#ifndef PREFETCHER_TRACE_DEBUG
#define PREFETCHER_TRACE_DEBUG(fmt, args...) \
  DPRINTF(PickleDevicePrefetcherTrace, "%s: " fmt, name(), ##args)
#define PREFETCHER_WORK_TRACKER_DEBUG(fmt, args...) \
  DPRINTF(PickleDevicePrefetcherWorkTrackerDebug, "%s: " fmt, name(), ##args)
#endif

namespace gem5
{

class PrefetcherWorkTracker;
class WorkItem;

// NPB UA compile-time constants (from NPB3.4-OMP/UA/ua_data.f90).
namespace ua_constants
{
    constexpr uint64_t LX1     = 5;
    constexpr uint64_t LNJE    = 2;
    constexpr uint64_t NSIDES  = 6;
    constexpr uint64_t NXYZ    = LX1 * LX1 * LX1;                  // 125

    constexpr uint64_t IDEL_ELEMS_PER_IE   = LX1 * LX1 * NSIDES;   // 150
    constexpr uint64_t IDMO_ELEMS_PER_IE   = LX1 * LX1 * LNJE
                                              * LNJE * NSIDES;     // 600
    constexpr uint64_t IDMO_ELEMS_PER_FACE = LX1 * LX1 * LNJE
                                              * LNJE;              // 100

    constexpr uint64_t IDX_ITEM_SIZE   = 4;     // int32  (Fortran default int)
    constexpr uint64_t LEAF_ITEM_SIZE  = 8;     // double precision
    constexpr uint64_t CBC_ITEM_SIZE   = 4;     // int32  (cbc is integer)

    constexpr uint64_t IDEL_BYTES_PER_IE = IDEL_ELEMS_PER_IE * IDX_ITEM_SIZE;
    constexpr uint64_t IDMO_BYTES_PER_IE = IDMO_ELEMS_PER_IE * IDX_ITEM_SIZE;
    constexpr uint64_t IDMO_BYTES_PER_FACE =
      IDMO_ELEMS_PER_FACE * IDX_ITEM_SIZE;
    constexpr uint64_t TX_BYTES_PER_IE = NXYZ * LEAF_ITEM_SIZE;
    constexpr uint64_t CBC_BYTES_PER_IE = NSIDES * CBC_ITEM_SIZE;
} // namespace ua_constants

// ---------------------------------------------------------------------------
// UATransferDensePrefetchGenerator
// ---------------------------------------------------------------------------
// Used for the "tx" leaf of transf / transfb. No indirection: given ie,
// directly mark idel(:,:,:,ie) and tx(:,:,:,ie) as expected prefetches.
class UATransferDensePrefetchGenerator: public PrefetchGenerator
{
  public:
    UATransferDensePrefetchGenerator(
        std::string _name,
        const uint64_t _job_id, const uint64_t _core_id,
        const uint64_t _software_hint_distance,
        const uint64_t _prefetch_distance_offset_from_software_hint,
        PrefetcherWorkTracker* _work_tracker
    );

    std::shared_ptr<WorkItem> execute_kernel(Addr work_data) override;
}; // class UATransferDensePrefetchGenerator

// ---------------------------------------------------------------------------
// UATransferMortarPrefetchGenerator
// ---------------------------------------------------------------------------
// Mortar gather/scatter with optional branch specialization (see the file
// header comment for the three-mode dispatch).
class UATransferMortarPrefetchGenerator: public PrefetchGenerator
{
  public:
    UATransferMortarPrefetchGenerator(
        std::string _name,
        const uint64_t _job_id, const uint64_t _core_id,
        const uint64_t _software_hint_distance,
        const uint64_t _prefetch_distance_offset_from_software_hint,
        // transf or transfb or transfb_c or transfb_c_2
        const std::string _function,
        const bool _cbc_optimization_enabled,
        PrefetcherWorkTracker* _work_tracker
    );

    std::shared_ptr<WorkItem> execute_kernel(Addr work_data) override;

  private:
    CbcMode cbc_mode;

    // Emit prefetches for every nonzero ig in [face_block_base,
    // face_block_base + IDMO_BYTES_PER_FACE).
    bool emitFaceFull(
        uint64_t                   element_id,
        uint64_t                   face_idx,
        Addr                       face_block_base,
        Addr                       leaf_base,
        std::shared_ptr<WorkItem>& workItem,
        uint64_t                   idmo_level,
        uint64_t                   leaf_level
    );

    // Emit prefetches for a conforming face. `nc_edge_emits_work` selects
    // between Transf semantics (nc edge -> 10 prefetches) and TransfbC
    // semantics (nc edge -> 0 prefetches).
    bool emitFaceConforming(
        uint64_t                   element_id,
        uint64_t                   face_idx,
        Addr                       face_block_base,
        Addr                       leaf_base,
        bool                       nc_edge_emits_work,
        std::shared_ptr<WorkItem>& workItem,
        uint64_t                   idmo_level,
        uint64_t                   leaf_level
    );

    // Read cbc(:, ie) into cbc_row. Returns false on load failure.
    bool readCbcRow(
        uint64_t                   element_id,
        int32_t                    cbc_row[ua_constants::NSIDES],
        std::shared_ptr<WorkItem>& workItem,
        uint64_t                   cbc_level
    );
}; // class UATransferMortarPrefetchGenerator

class UANumElementsUpdateKernel: public PrefetchGenerator
{
  public:
    UANumElementsUpdateKernel(
        std::string _name,
        const uint64_t _job_id, const uint64_t _core_id,
        const uint64_t _software_hint_distance,
        const uint64_t _prefetch_distance_offset_from_software_hint,
        PrefetcherWorkTracker* _work_tracker
    );

    // Function to update the prefetch context
    std::shared_ptr<WorkItem> execute_kernel(Addr work_data) override;
}; // class UANumElementsUpdateKernel

} // namespace gem5

#endif // __UA_PREFETCH_GENERATOR_HH__
