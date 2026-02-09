/*
 * Copyright (c) 2026 The Regents of the University of California
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __DMP_INTERFACE_HH__
#define __DMP_INTERFACE_HH__

#include "enums/CacheLevel.hh"

namespace gem5
{

namespace prefetch
{

namespace dmp
{

enum class AccessType
{
    Single,
    Range
};

class DifferentialMatchingPrefetcherInterface
{
  public:
    virtual ~DifferentialMatchingPrefetcherInterface() = default;
    // Called when a new stride is detected, the parameter is PC of the
    // instruction.
    virtual void handleNewlyDetectedStride(const Addr pc) = 0;
    // Called when the ICS still has capacity to accommodate new candidate
    // index PC.
    virtual void handleIcsHasAvailableSlots() = 0;
    // Called when a candidate pair of PCs is promoted from the Indirection
    // Candidate Scoreboard (ICS), the parameter is PC of the instruction.
    virtual void handleNewCandidateFromIcs(
      const Addr index_pc, const Addr target_pc
    ) = 0;
    // Called when a differential matching result is available for a candidate
    // pair of PCs.
    virtual void handleDifferentialMatchResult(
      const Addr index_pc, const Addr target_pc, const bool successful_match,
      const Addr target_base_vaddr, const int64_t shift_amount,
      const AccessType index_access_type, const AccessType target_access_type
    ) = 0;
    // Notify the prefetch proxy of a new prefetch request, triggering the
    // prefetch proxy to schedule prefetch requests.
    // This is called by the prefetch queue when a new prefetch request is
    // enqueued.
    virtual void notifyNewPrefetchRequest(
      const enums::CacheLevel cache_controller_level
    ) = 0;
    // Notify the dmp of the prefetched data from stride prefetcher
    virtual void handleNewPrefetchedDataFromStridePrefetcher(
      const Addr target_paddr, const Addr pc, const uint64_t data
    ) = 0;
    // Allow other prefetchers to query whether DMP already prefetches for
    // a specific PC.
    virtual bool isATargetPC(const Addr pc) const = 0;
};

} // namespace dmp

} // namespace prefetch

} // namespace gem5

#endif // __DMP_INTERFACE_HH__
