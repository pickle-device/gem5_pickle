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

#ifndef __SSSP_PREFETCH_GENERATOR_HH__
#define __SSSP_PREFETCH_GENERATOR_HH__

#include <memory>
#include <string>

#include "base/logging.hh"
#include "debug/PickleDevicePrefetcherTrace.hh"
#include "debug/PickleDevicePrefetcherWorkTrackerDebug.hh"
#include "pickle/application_specific/prefetcher/prefetch_generators/prefetch_generator.hh"

#define PREFETCHER_TRACE_DEBUG(fmt, args...) \
  DPRINTF(PickleDevicePrefetcherTrace, "%s: " fmt, name(), ##args)
#define PREFETCHER_WORK_TRACKER_DEBUG(fmt, args...) \
  DPRINTF(PickleDevicePrefetcherWorkTrackerDebug, "%s: " fmt, name(), ##args)

namespace gem5
{

class PrefetcherWorkTracker;

class SSSPPrefetchKernel1Generator: public PrefetchGenerator
{
  public:
    SSSPPrefetchKernel1Generator(
        std::string _name,
        const uint64_t _job_id, const uint64_t _core_id,
        const uint64_t _software_hint_distance,
        const uint64_t _prefetch_distance_offset_from_software_hint,
        const bool _sssp_threshold_optimization_enabled,
        PrefetcherWorkTracker* _work_tracker
    );

    // Function to generate prefetch requests
    std::shared_ptr<WorkItem> execute_kernel(Addr work_data) override;
  private:
    bool sssp_threshold_optimization_enabled;
}; // class SSSPPrefetchKernel1Generator

class SSSPPrefetchKernel2Generator: public PrefetchGenerator
{
  public:
    SSSPPrefetchKernel2Generator(
        std::string _name,
        const uint64_t _job_id, const uint64_t _core_id,
        const uint64_t _software_hint_distance,
        const uint64_t _prefetch_distance_offset_from_software_hint,
        PrefetcherWorkTracker* _work_tracker
    );

    // Function to generate prefetch requests
    std::shared_ptr<WorkItem> execute_kernel(Addr work_data) override;
}; // class SSSPPrefetchKernel2Generator

class SSSPPrefetchKernel3Generator: public PrefetchGenerator
{
  public:
    SSSPPrefetchKernel3Generator(
        std::string _name,
        const uint64_t _job_id, const uint64_t _core_id,
        const uint64_t _software_hint_distance,
        const uint64_t _prefetch_distance_offset_from_software_hint,
        PrefetcherWorkTracker* _work_tracker
    );

    // Function to update the prefetch context
    std::shared_ptr<WorkItem> execute_kernel(Addr work_data) override;
}; // class SSSPPrefetchKernel3Generator

} // namespace gem5

#endif // __SSSP_PREFETCH_GENERATOR_HH__
