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
// NAS UA calls transf / transfb twice per CG iteration and transfb_c /
// transfb_c_2 once per timestep (see diffuse.f90, convect.f90). All four
// sweeps share the same skeleton:
//
//     do ie = 1, nelt                              ! driver
//       do iface = 1, nsides                       ! scatter-gather into:
//         il = idel(..., iface, ie)                !   level-1 index -> tx
//         ig = idmo(..., iface, ie)                !   level-1 index -> tmor
//         tx  (il) = ...                           !   dense leaf (cube/elem)
//         tmor(ig) = ...                           !   sparse leaf (mortar)
//       end do
//     end do
//
// Two distinct prefetch behaviors are useful here:
//
//   (1) Dense leaf (tx): per-ie the touched region is a contiguous
//       lx1^3 * sizeof(double) = 1 KiB cube. We do not need to read
//       idel first; we can directly emit expected prefetches for the
//       whole range. Paired with prefetching the idel chunk itself so
//       the scalar idel loads hit in L1.
//
//   (2) Sparse leaf (tmor / tmort): per-ie the idmo chunk holds up to
//       lx1*lx1*lnje*lnje*nsides = 600 int32 indices that scatter into
//       the mortar vector. We issue a zero-cycle read on the idmo
//       chunk, extract each nonzero ig, and emit an expected prefetch
//       for the cache line containing tmor[ig]. This is the classic
//       PR-style two-level chase applied to the mortar mesh structure.
//
// Two classes. Six kernels:
//
//   "ua_transf_tx"     -> UATransferDensePrefetchGenerator   (slot 0)
//   "ua_transf_tmor"   -> UATransferMortarPrefetchGenerator  (slot 1)
//   "ua_transfb_tx"    -> UATransferDensePrefetchGenerator   (slot 2)
//   "ua_transfb_tmor"  -> UATransferMortarPrefetchGenerator  (slot 3)
//   "ua_transfb_c"     -> UATransferMortarPrefetchGenerator  (slot 4)
//   "ua_transfb_c2"    -> UATransferMortarPrefetchGenerator  (slot 5)
//
// The kernel_name -> class mapping lives in the prefetch generator
// factory (outside this file, wherever PR/BFS/SSSP/TC/BC/CC are
// registered). The factory is expected to pass
// cbc_skip_optimization_enabled = true for slots 4/5 (transfb_c,
// transfb_c_2) and false for slots 1/3 (transf, transfb) to match the
// application-level `cbc(iface,ie).ne.3` guard. See sssp.hh for the
// analogous `sssp_threshold_optimization_enabled` flag.
//
// ---------------------------------------------------------------------------
// Job descriptor layout (as emitted by pickle_ua_glue.cc)
// ---------------------------------------------------------------------------
//   Array 0: level-1 index
//              for *_tx       : idel     (int32, Ranged, Index)
//              for *_tmor/tmort: idmo    (int32, Ranged, Index)
//   Array 1: leaf
//              for *_tx       : tx                (double, SingleElement)
//              for *_tmor     : tmor              (double, SingleElement)
//              for *_transfb_c[.2]: tmort         (double, SingleElement)
//
// Optional (future): Array 2: cbc (int32, Ranged) for cbc_skip opt.
//                    Array 3: mormult (double, SingleElement) for slot 5.
//
// ---------------------------------------------------------------------------
// Work data semantics
// ---------------------------------------------------------------------------
// The application writes `ie - 1` (zero-based element index) to the UCPage.
// This mirrors pr2.cc's convention (index-based work_data). We therefore
// compute:
//
//     element_id = work_data + software_hint_distance
//                            - prefetch_distance_offset_from_software_hint
//
// (no `* item_size` — that is the BFS/SSSP iterator-based form.)
//
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

// NPB UA compile-time constants (from NPB3.4-OMP/UA/ua_data.f90).
// Hard-coded rather than carried in the job descriptor because they are
// compile-time parameters for the application and never vary per run.
namespace ua_constants
{
    constexpr uint64_t LX1    = 5;   // Gauss-Lobatto points per direction
    constexpr uint64_t LNJE   = 2;   // mortar pieces per nonconforming face
    constexpr uint64_t NSIDES = 6;   // element faces
    constexpr uint64_t NXYZ   = LX1 * LX1 * LX1;  // = 125

    // Elements counts per ie (chunk strides of the Ranged level-1 arrays)
    constexpr uint64_t IDEL_ELEMS_PER_IE = LX1 * LX1 * NSIDES;  //  150
    constexpr uint64_t IDMO_ELEMS_PER_IE = LX1 * LX1 * LNJE * LNJE *
                                           NSIDES;  //  600

    // Fortran default integer is 4 bytes, double precision is 8 bytes
    constexpr uint64_t IDX_ITEM_SIZE     = 4;      // idel, idmo element size
    constexpr uint64_t LEAF_ITEM_SIZE    = 8;      // tx, tmor, tmort elem size

    // Bytes per ie
    constexpr uint64_t IDEL_BYTES_PER_IE = IDEL_ELEMS_PER_IE * IDX_ITEM_SIZE;
    constexpr uint64_t IDMO_BYTES_PER_IE = IDMO_ELEMS_PER_IE * IDX_ITEM_SIZE;
    constexpr uint64_t TX_BYTES_PER_IE   = NXYZ              * LEAF_ITEM_SIZE;
} // namespace ua_constants

// ---------------------------------------------------------------------------
// UATransferDensePrefetchGenerator
// ---------------------------------------------------------------------------
// Used for the "tx" leaf of transf / transfb. No indirection: given ie,
// mark idel(:,:,:,ie) and tx(:,:,:,ie) as expected prefetches.
//
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
// Used for the "tmor" / "tmort" leaves of transf / transfb / transfb_c /
// transfb_c_2. Two-level chase: ie -> idmo(:,:,:,:,:,ie) chunk -> tmor[ig].
//
// cbc_skip_optimization_enabled controls whether we honor the application's
// `if (cbc(iface,ie) .ne. 3)` guard used in transfb_c / transfb_c_2. When
// enabled, a cbc array must be supplied at array_index = 2 of the job
// descriptor; the generator will read cbc(:, ie) and mask out nonconforming
// faces. When disabled (the transf / transfb case), we emit prefetches for
// every face regardless of cbc.
//
class UATransferMortarPrefetchGenerator: public PrefetchGenerator
{
  public:
    UATransferMortarPrefetchGenerator(
        std::string _name,
        const uint64_t _job_id, const uint64_t _core_id,
        const uint64_t _software_hint_distance,
        const uint64_t _prefetch_distance_offset_from_software_hint,
        const bool     _cbc_skip_optimization_enabled,
        PrefetcherWorkTracker* _work_tracker
    );

    std::shared_ptr<WorkItem> execute_kernel(Addr work_data) override;

  private:
    bool cbc_skip_optimization_enabled;
}; // class UATransferMortarPrefetchGenerator

} // namespace gem5

#endif // __UA_PREFETCH_GENERATOR_HH__
