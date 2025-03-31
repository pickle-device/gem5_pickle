// Copyright (c) 2025 The Regents of the University of California
// All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef __MEM_CACHE_SIMPLE_PROBE_ARG_HH__
#define __MEM_CACHE_SIMPLE_PROBE_ARG_HH__

#include "mem/packet.hh"
#include "mem/ruby/common/MachineID.hh"

namespace gem5
{

struct SimpleCacheAccessor
{
};

class SimpleCacheAccessProbeArg
{
  public:
    RequestPtr req;
    SimpleCacheAccessor &cache;
    ruby::MachineID machineID;
    bool machineIDValid;
    Tick latency;
    SimpleCacheAccessProbeArg(
        RequestPtr _req, SimpleCacheAccessor& _cache,
        ruby::MachineID _machineID, bool _machineIDValid,
        Tick _latency
    ) : req(_req), cache(_cache), machineID(_machineID),
        machineIDValid(_machineIDValid), latency(_latency)
    {
    }
};

} // namespace gem5

#endif //__MEM_SIMPLE_CACHE_PROBE_ARG_HH__
