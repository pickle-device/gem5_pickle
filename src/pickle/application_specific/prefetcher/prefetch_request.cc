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

#include "pickle/application_specific/prefetcher/prefetch_request.hh"

#include <cassert>

namespace gem5
{

PrefetchRequest::PrefetchRequest()
    : pf_vaddr(-1ULL), pf_req_time(-1ULL), pf_id(-1ULL), has_paddr(false),
      is_delegated_to_prefetch_agent(false)
{
}

PrefetchRequest
PrefetchRequest::createWithVAddr(
    Addr pf_vaddr, Tick pf_req_time, uint64_t pf_id,
    bool is_delegated_to_prefetch_agent
)
{
    PrefetchRequest request;
    request.pf_vaddr = pf_vaddr;
    request.pf_req_time = pf_req_time;
    request.pf_id = pf_id;
    request.has_paddr = false;
    request.is_delegated_to_prefetch_agent = is_delegated_to_prefetch_agent;
    return request;
}

PrefetchRequest
PrefetchRequest::createWithPAddr(
    Addr pf_paddr, Addr pf_vaddr, Tick pf_req_time, uint64_t pf_id,
    bool is_delegated_to_prefetch_agent
)
{
    PrefetchRequest request;
    request.pf_paddr = pf_paddr;
    request.pf_vaddr = pf_vaddr;
    request.pf_req_time = pf_req_time;
    request.pf_id = pf_id;
    request.has_paddr = true;
    request.is_delegated_to_prefetch_agent = is_delegated_to_prefetch_agent;
    return request;
}

Addr
PrefetchRequest::getPrefetchVAddr() const
{
    return pf_vaddr;
}

Addr
PrefetchRequest::getPrefetchPAddr() const
{
    assert(has_paddr);
    return pf_paddr;
}

void
PrefetchRequest::setPrefetchPAddr(Addr pf_paddr)
{
    this->pf_paddr = pf_paddr;
    this->has_paddr = true;
}

bool
PrefetchRequest::hasPAddr() const
{
    return has_paddr;
}

Tick
PrefetchRequest::getPrefetchReqTime() const
{
    return pf_req_time;
}

uint64_t
PrefetchRequest::getPrefetchId() const
{
    return pf_id;
}

bool
PrefetchRequest::isDelegatedToPrefetchAgent() const
{
    return is_delegated_to_prefetch_agent;
}

}; // namespace gem5
