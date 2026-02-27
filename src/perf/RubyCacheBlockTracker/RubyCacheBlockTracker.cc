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

#include "perf/RubyCacheBlockTracker/RubyCacheBlockTracker.hh"

#include "base/statistics.hh"
#include "base/stats/group.hh"
#include "base/types.hh"
#include "debug/RubyCacheBlockTrackerDebug.hh"
#include "mem/cache/simple_cache_probe_arg.hh"
#include "mem/request.hh"
#include "params/RubyCacheBlockTracker.hh"
#include "sim/probe/probe.hh"
#include "sim/sim_object.hh"
#include "sim/system.hh"

namespace gem5
{

namespace ruby
{

RubyCacheBlockTracker::RubyCacheBlockTracker(const Params &p)
  : ProbeListenerObject(p),
    system(p.system),
    usefulnessAttributionStats(this)
{
}

RubyCacheBlockTracker::~RubyCacheBlockTracker()
{
}

void
RubyCacheBlockTracker::init()
{
    const uint64_t num_requestors = system->maxRequestors();
    RUBY_CACHE_BLOCK_TRACKER_DEBUG(
        "RubyCacheBlockTracker initialized with %d requestors\n",
        num_requestors
    );
    for (RequestorID id = 0; id < num_requestors; ++id) {
        std::string requestor_name = system->getRequestorName(id);
        RUBY_CACHE_BLOCK_TRACKER_DEBUG(
            "Registering for requestor id %d with name %s\n",
            id, requestor_name.c_str()
        );
    }
}

void
RubyCacheBlockTracker::addEventProbe(SimObject *obj, const char *event_name)
{
    ProbeManager *obj_pm = obj->getProbeManager();
    if (strcmp(event_name, "cpu outgoing data request") == 0) {
        listeners.push_back(new CpuRequestListener(this, obj_pm, event_name));
        obj_pm->addListener(event_name, *(listeners.back()));
    } else if (strcmp(event_name, "Directory entry allocation") == 0) {
        listeners.push_back(
            new DirEntryAllocationListener(this, obj_pm, event_name)
        );
        obj_pm->addListener(event_name, *(listeners.back()));
    } else if (strcmp(event_name, "Directory entry deallocation") == 0) {
        listeners.push_back(
            new DirEntryDeallocationListener(this, obj_pm, event_name)
        );
        obj_pm->addListener(event_name, *(listeners.back()));
    } else if (strcmp(event_name, "DataMovementWriteback") == 0) {
        listeners.push_back(
            new DataMovementListener(this, obj_pm, event_name, true, false)
        );
        obj_pm->addListener(event_name, *(listeners.back()));
    } else if (strcmp(event_name, "DataMovementEviction") == 0) {
        listeners.push_back(
            new DataMovementListener(this, obj_pm, event_name, false, true)
        );
        obj_pm->addListener(event_name, *(listeners.back()));
    } else {
        panic(
            "Unsupported event name for RubyCacheBlockTracker: %s", event_name
        );
    }
}

void
RubyCacheBlockTracker::processCpuRequest(const RequestPtr &req)
{
    // TODO
}

RubyCacheBlockTracker::
UsefulnessAttributionStats::UsefulnessAttributionStats(
    statistics::Group *parent
) : statistics::Group(parent)
{
}


RubyCacheBlockTracker::
CpuRequestListener::CpuRequestListener(
    RubyCacheBlockTracker *_owner, ProbeManager *_probe_manager,
    const char *_name
) : ProbeListenerArgBase<RequestPtr>(_probe_manager, _name), owner(_owner)
{
}

void
RubyCacheBlockTracker::CpuRequestListener::notify(const RequestPtr &req)
{
    owner->processCpuRequest(req);
}

RubyCacheBlockTracker::DirEntryAllocationListener::DirEntryAllocationListener(
    RubyCacheBlockTracker *_owner, ProbeManager *_probe_manager,
    const char *_name
) : ProbeListenerArgBase<std::pair<Addr, RequestPtr>>(_probe_manager, _name),
    owner(_owner)
{
}

void
RubyCacheBlockTracker::DirEntryAllocationListener::notify(
    const std::pair<Addr, RequestPtr> &arg
)
{
    // TODO
}

RubyCacheBlockTracker::
DirEntryDeallocationListener::DirEntryDeallocationListener(
    RubyCacheBlockTracker *_owner, ProbeManager *_probe_manager,
    const char *_name
) : ProbeListenerArgBase<Addr>(_probe_manager, _name), owner(_owner)
{
}

void
RubyCacheBlockTracker::DirEntryDeallocationListener::notify(const Addr &arg)
{
    // TODO
}

RubyCacheBlockTracker::DataMovementListener::DataMovementListener(
    RubyCacheBlockTracker *_owner, ProbeManager *_probe_manager,
    const char *_name, const bool _is_cache_fill, const bool _is_cache_evict
) : ProbeListenerArgBase<SimpleCacheAccessProbeArg>(_probe_manager, _name),
    owner(_owner), is_cache_fill(_is_cache_fill),
    is_cache_evict(_is_cache_evict)
{
    panic_if(
        is_cache_fill && is_cache_evict,
        "DataMovementListener cannot be both cache fill and cache evict"
    );
    panic_if(!is_cache_fill && !is_cache_evict,
        "DataMovementListener must be either cache fill or cache evict"
    );
}

void
RubyCacheBlockTracker::DataMovementListener::notify(
    const SimpleCacheAccessProbeArg &arg
)
{
    // TODO
}

}  // namespace ruby

}  // namespace gem5
