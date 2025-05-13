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

namespace gem5
{

PrefetchRequest::PrefetchRequest()
    : pf_vaddr(-1ULL), pf_req_time(-1ULL)
{
}

PrefetchRequest::PrefetchRequest(const Addr pf_vaddr, const Tick pf_req_time)
    : pf_vaddr(pf_vaddr), pf_req_time(pf_req_time)
{
}

Addr
PrefetchRequest::getPrefetchVAddr() const
{
    return pf_vaddr;
}

Tick
PrefetchRequest::getPrefetchReqTime() const
{
    return pf_req_time;
}

//bool
//PrefetchRequest::operator==(const PrefetchRequest& other) const
//{
//    return pf_vaddr == other.pf_vaddr;
//}
//
//bool
//PrefetchRequest::operator<(const PrefetchRequest& other) const
//{
//    // HACK: we want to sort the queue in ascending order
//    // so we need to reverse the comparison
//    return pf_req_time >= other.pf_req_time;
//}

}; // namespace gem5
