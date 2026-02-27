/*
 * Copyright (c) 2026 The Regents of the University of California
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "mem/ruby/structures/RubyDirEntryTrackerProxy.hh"

#include <utility>

#include "base/named.hh"
#include "base/trace.hh"
#include "debug/RubyDirEntryTrackerProxyDebug.hh"
#include "mem/request.hh"
#include "mem/ruby/slicc_interface/AbstractController.hh"

namespace gem5
{

namespace ruby
{

RubyDirEntryTrackerProxy::RubyDirEntryTrackerProxy(
    AbstractController* _cacheController
) : Named(_cacheController->name() + ".DirEntryTrackerProxy"),
    cacheController(_cacheController)
{}

RubyDirEntryTrackerProxy::~RubyDirEntryTrackerProxy()
{
    delete ppDirEntryAllocation;
    delete ppDirEntryDeallocation;
}

void
RubyDirEntryTrackerProxy::regProbePoints()
{
    ppDirEntryAllocation = new ProbePointArg<std::pair<Addr, RequestPtr>>(
        cacheController->getProbeManager(), "Directory entry allocation"
    );
    ppDirEntryDeallocation = new ProbePointArg<Addr>(
        cacheController->getProbeManager(), "Directory entry deallocation"
    );
}

void
RubyDirEntryTrackerProxy::notifyDirEntryAllocation(
    const Addr paddr, const RequestPtr request
)
{
    ppDirEntryAllocation->notify(std::make_pair(paddr, request));
    DPRINTF(
        RubyDirEntryTrackerProxyDebug,
        "Dir Entry ALLOC: paddr: %#x, RequestorId: %lld\n",
        paddr, request->requestorId()
    );
}

void
RubyDirEntryTrackerProxy::notifyDirEntryDeallocation(const Addr paddr)
{
    ppDirEntryDeallocation->notify(paddr);
    DPRINTF(
        RubyDirEntryTrackerProxyDebug,
        "Dir Entry DEALLOC: paddr: %#x\n", paddr
    );
}

} // namespace ruby

} // namespace gem5
