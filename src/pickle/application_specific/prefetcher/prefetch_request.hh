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

#ifndef __PREFETCH_REQUEST_HH__
#define __PREFETCH_REQUEST_HH__

#include "base/types.hh"

namespace gem5
{

class PrefetchRequest
{
    private:
        Addr pf_vaddr;
        Addr pf_paddr;
        ContextID pf_context_id;
        Tick pf_req_time;
        uint64_t pf_id;
        bool has_paddr;
        bool is_delegated_to_prefetch_agent;
        uint64_t priority_score;
        // When the prefetch request is enqueued to the prefetch agent, we
        // record the current tick as the start time. If the request is still
        // not completed after some timeout threshold, the prefetch agent can
        // retry the request. This field is used to record the start time for
        // timeout calculation.
        Tick timeout;
    public:
        PrefetchRequest();
        static PrefetchRequest createWithVAddr(
            Addr pf_vaddr, ContextID context_id, Tick pf_req_time,
            uint64_t pf_id, bool is_delegated_to_prefetch_agent,
            uint64_t priority_score
        );
        static PrefetchRequest createWithPAddr(
            Addr pf_paddr, Addr pf_vaddr, Tick pf_req_time,
            uint64_t pf_id, bool is_delegated_to_prefetch_agent,
            uint64_t priority_score
        );
        Addr getPrefetchVAddr() const;
        Addr getPrefetchPAddr() const;
        void setPrefetchPAddr(Addr pf_paddr);
        ContextID getPrefetchContextID() const;
        bool hasPAddr() const;
        Tick getPrefetchReqTime() const;
        void setTimeout(const Tick timeout);
        bool isTimedOut(const Tick current_tick) const;
        // the higher the priority, the earlier the request is issued
        uint64_t getPrefetchPriorityScore() const;
        uint64_t getPrefetchId() const;
        bool isDelegatedToPrefetchAgent() const;
};


struct PrefetchRequestOrder
{
    bool operator()(PrefetchRequest const& a, PrefetchRequest const& b) const
    {
        // We are using this with std::priority_queue, which puts the largest
        // element on top. We want the request with the highest priority score
        // to be issued first, so we return true if a's priority score is less
        // than b's priority score.
        return a.getPrefetchPriorityScore() < b.getPrefetchPriorityScore();
    }
};

}; // namespace gem5

#endif // __PREFETCH_REQUEST_HH__
