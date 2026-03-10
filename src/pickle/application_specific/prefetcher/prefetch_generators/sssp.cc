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
    // TODO
    return nullptr;
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
    // TODO
    return nullptr;
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
