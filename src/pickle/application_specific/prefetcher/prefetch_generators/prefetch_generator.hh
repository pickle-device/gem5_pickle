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

#ifndef __PREFETCH_GENERATOR_HH__
#define __PREFETCH_GENERATOR_HH__

#include <memory>
#include <string>

#include "pickle/application_specific/prefetcher/work_item.hh"

namespace gem5
{

class PrefetcherWorkTracker;

class PrefetchGenerator
{
  protected:
    std::string generator_name;
    uint64_t software_hint_distance;
    uint64_t prefetch_distance_offset_from_software_hint;
    PrefetcherWorkTracker* work_tracker;
  public:
    PrefetchGenerator(
        std::string _name,
        const uint64_t _software_hint_distance,
        const uint64_t _prefetch_distance_offset_from_software_hint,
        PrefetcherWorkTracker* _work_tracker
    );

    std::string name() const;

    // Function to generate prefetch requests
    virtual std::shared_ptr<WorkItem> generateWorkItem(Addr work_data) = 0;

    void warnIfOutsideRanges(
      const Addr work_id, const Addr pf_vaddr
    ) const;
}; // class PrefetchGenerator

} // namespace gem5

#endif // __PREFETCH_GENERATOR_HH__
