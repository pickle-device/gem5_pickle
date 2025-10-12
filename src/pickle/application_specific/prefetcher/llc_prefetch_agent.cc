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

#include "pickle/application_specific/prefetcher/llc_prefetch_agent.hh"

#include <cassert>
#include <utility>

namespace gem5
{

LLCPrefetchAgent::LLCPrefetchAgent(const LLCPrefetchAgentParams &params)
    : ClockedObject(params),
      llc_controller(params.llc_controller),
      addr_ranges(params.addr_ranges),
      agent_stats(this)
{
    assert(llc_controller != nullptr);
}

LLCPrefetchAgent::~LLCPrefetchAgent()
{
}

void LLCPrefetchAgent::setPicklePrefetcher(PicklePrefetcher* prefetcher)
{
    assert(prefetcher != nullptr);
    this->prefetcher = prefetcher;
}

void LLCPrefetchAgent::enqueueRequestWithPAddr(PrefetchRequest request)
{
    assert(request.hasPAddr());
    agent_stats.prefetch_request_count++;
    prefetch_request_queue.push(std::move(request));
    agent_stats.prefetch_request_queue_length.sample(
        prefetch_request_queue.size()
    );
}

LLCPrefetchAgent::LLCPrefetchAgentStats::LLCPrefetchAgentStats(
    statistics::Group *parent)
    : statistics::Group(parent, "llc_prefetch_agent"),
      ADD_STAT(prefetch_request_count, statistics::units::Count::get(),
               "Number of prefetch requests received by the prefetcher"),
      ADD_STAT(prefetch_request_dropped_due_to_cache_line_presence,
               statistics::units::Count::get(),
               "Number of prefetch requests dropped due to the cache line "
               "already being present in the cache"),
      ADD_STAT(prefetch_request_sent, statistics::units::Count::get(),
               "Number of prefetch requests sent to the memory system"),
      ADD_STAT(prefetch_request_queue_length, statistics::units::Count::get(),
                "Histogram of the prefetch request queue length over time")
{
}

void LLCPrefetchAgent::LLCPrefetchAgentStats::regStats()
{
    statistics::Group::regStats();
    prefetch_request_queue_length
        .init(16)
        .flags(statistics::pdf);
}

}; // namespace gem5
