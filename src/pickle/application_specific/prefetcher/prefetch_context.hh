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

#include <cstdint>
#include <unordered_map>

#ifndef __PREFETCH_CONTEXT_HH__
#define __PREFETCH_CONTEXT_HH__

namespace gem5
{

class PicklePrefetcher;

// The context that the prefetcher can use to make prefetching decisions.
// In hardware, the context can be implemented as a dictionary of key-value
// pairs, and the prefetcher can query the value of a specific key.
// However, for the sake of maintainability and readability, we implement the
// context as a class with specific getter functions for different fields for
// different workloads. For example, in SSSP, if the prefetcher needs to know
// the current distance threshold to relax the edges, we can have a
// getCurrentDistanceThreshold() function in the PrefetchContext class. This
// way, we can avoid the prefetcher having to know about the specific key names
// in the dictionary, and we can also have type safety for the values returned
// by the getter functions.

class PrefetchContext
{
  private:
    PicklePrefetcher* owner;
    // The current distance threshold to relax the edges in SSSP.
    std::unordered_map<uint64_t, uint64_t> sssp_current_distance_threshold;
    // The current depth in BC.
    std::unordered_map<uint64_t, uint64_t> bc_current_depth;
    // Num elements in UA.
    std::unordered_map<uint64_t, uint64_t> ua_num_elements;
  public:
    PrefetchContext();
    void setOwner(PicklePrefetcher* _owner);
    uint64_t getSSSPCurrentDistanceThreshold(uint64_t core_id) const;
    void setSSSPCurrentDistanceThreshold(uint64_t core_id, uint64_t threshold);
    uint64_t getBCCurrentDepth(uint64_t core_id) const;
    void setBCCurrentDepth(uint64_t core_id, uint64_t depth);
    uint64_t getUANumElements(uint64_t core_id) const;
    void setUANumElements(uint64_t core_id, uint64_t num_elements);
};  // class PrefetchContext

}; // namespace gem5

#endif // __PREFETCH_CONTEXT_HH__
