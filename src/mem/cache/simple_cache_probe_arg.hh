// Copyright (c) 2025 The Regents of the University of California
// All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef __MEM_CACHE_SIMPLE_PROBE_ARG_HH__
#define __MEM_CACHE_SIMPLE_PROBE_ARG_HH__

#include <utility>
#include <vector>

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
    std::vector<uint8_t> cache_fill_data;
    SimpleCacheAccessor &cache;
    ruby::MachineID machineID;
    bool machineIDValid;
    Tick latency;
    unsigned cache_state;
    SimpleCacheAccessProbeArg(
        RequestPtr _req, SimpleCacheAccessor& _cache,
        ruby::MachineID _machineID, bool _machineIDValid,
        Tick _latency, unsigned _cache_state,
        std::vector<uint8_t> _cache_fill_data
    ) : req(_req), cache_fill_data(std::move(_cache_fill_data)),
        cache(_cache), machineID(_machineID),
        machineIDValid(_machineIDValid), latency(_latency),
        cache_state(_cache_state)
    {
    }
    bool hasCacheFillData() const
    {
        return !cache_fill_data.empty();
    }
};

} // namespace gem5

#endif //__MEM_SIMPLE_CACHE_PROBE_ARG_HH__
