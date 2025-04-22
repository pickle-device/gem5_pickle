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

#include "pickle/application_specific/prefetcher/prefetch_generators/prefetch_generator.hh"

#include "debug/PickleDevicePrefetcherKnownBugs.hh"
#include "pickle/application_specific/prefetcher/prefetcher_interface.hh"

namespace gem5
{

PrefetchGenerator::PrefetchGenerator(
    std::string _name,
    const uint64_t _prefetch_distance_offset_from_software_hint,
    PrefetcherWorkTracker* _work_tracker
) : generator_name(_name),
    prefetch_distance_offset_from_software_hint(
        _prefetch_distance_offset_from_software_hint
    ),
    work_tracker(_work_tracker)
{
}

std::string
PrefetchGenerator::name() const
{
    return generator_name;
}

void
PrefetchGenerator::warnIfOutsideRanges(
    const Addr work_id, const Addr pf_vaddr
) const
{
    uint64_t array_id = work_tracker->job_descriptor->get_array_id(pf_vaddr);
    if (array_id == -1ULL) {
        work_tracker->owner->profilePrefetchWithUnknownVAddr();
        DPRINTF(
            PickleDevicePrefetcherKnownBugs,
            "warn: unknown pf_vaddr: work_id=0x%llx, pf_vaddr=0x%llx\n",
            work_id, pf_vaddr
        );
    }
}

} // namespace gem5
