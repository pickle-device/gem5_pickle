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

#ifndef __LLC_PREFETCH_AGENT_HH__
#define __LLC_PREFETCH_AGENT_HH__

#include <queue>
#include <vector>

#include "base/addr_range.hh"
#include "mem/ruby/protocol/CHI/Cache_Controller.hh"
#include "params/LLCPrefetchAgent.hh"
#include "pickle/application_specific/prefetcher/pickle_prefetcher.hh"
#include "pickle/application_specific/prefetcher/prefetch_request.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class LLCPrefetchAgent: public ClockedObject
{
    private:
        PARAMS(LLCPrefetchAgent);
    private:
        PicklePrefetcher* prefetcher;
        ruby::CHI::Cache_Controller* llc_controller;
        std::vector<AddrRange> addr_ranges;
        std::priority_queue<
            PrefetchRequest, std::vector<PrefetchRequest>, PrefetchRequestOrder
        > prefetch_request_queue;
    public:
        LLCPrefetchAgent(const LLCPrefetchAgentParams &params);
        ~LLCPrefetchAgent();
        void setPicklePrefetcher(PicklePrefetcher* prefetcher);
        // Enqueue a prefetch request with a physical address.
        // We do not allow enqueuing a request with a virtual address, because
        // the LLC prefetch agent should only work with physical addresses.
        void enqueueRequestWithPAddr(PrefetchRequest request);
    public:
        struct LLCPrefetchAgentStats : public statistics::Group
        {
            LLCPrefetchAgentStats(statistics::Group *parent);
            void regStats() override;
            statistics::Scalar prefetch_request_count;
            statistics::Scalar \
                prefetch_request_dropped_due_to_cache_line_presence;
            statistics::Scalar prefetch_request_sent;
            statistics::Histogram prefetch_request_queue_length;
        } agent_stats;
};

}; // namespace gem5

#endif // __LLC_PREFETCH_AGENT_HH__
