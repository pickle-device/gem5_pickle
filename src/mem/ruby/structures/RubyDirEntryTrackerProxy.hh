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

#ifndef __RUBY_DIR_ENTRY_TRACKER_PROXY_HH__
#define __RUBY_DIR_ENTRY_TRACKER_PROXY_HH__

#include "base/named.hh"
#include "base/types.hh"
#include "mem/request.hh"
#include "mem/ruby/common/MachineID.hh"
#include "mem/ruby/slicc_interface/AbstractController.hh"
#include "sim/probe/probe.hh"

namespace gem5
{

namespace ruby
{

class RubyDirEntryTrackerProxy : public Named
{
  public:
    RubyDirEntryTrackerProxy(AbstractController* cacheController);
    ~RubyDirEntryTrackerProxy();
    void regProbePoints();

    // Notification for directory entry update events
    // The request parameter is the request from the sequencer, i.e., the
    // original request, not messages between ruby caches
    void notifyDirEntryAllocation(
        const Addr paddr, const RequestPtr request
    );
    void notifyDirEntryDeallocation(const Addr paddr);

  private:
    AbstractController* cacheController;

    ProbePointArg<std::pair<Addr, RequestPtr>> *ppDirEntryAllocation;
    ProbePointArg<Addr> *ppDirEntryDeallocation;
};

} // namespace ruby

} // namespace gem5

#endif // __RUBY_DIR_ENTRY_TRACKER_PROXY_HH__
