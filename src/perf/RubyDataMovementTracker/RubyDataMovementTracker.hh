/*
 * Copyright (c) 2024, 2025 The Regents of the University of California
 * All rights reserved
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __PERF_RUBY_DATA_MOVEMENT_TRACKER_HH__
#define __PERF_RUBY_DATA_MOVEMENT_TRACKER_HH__

#include <bitset>
#include <vector>

#include "base/circular_queue.hh"
#include "base/statistics.hh"
#include "mem/cache/simple_cache_probe_arg.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/network/MessageBuffer.hh"
#include "mem/ruby/slicc_interface/AbstractController.hh"
#include "mem/ruby/slicc_interface/RubyRequest.hh"
#include "mem/ruby/system/RubySystem.hh"
#include "params/RubyDataMovementTracker.hh"
#include "sim/probe/probe.hh"
#include "sim/sim_object.hh"

namespace gem5 {

namespace ruby {

class RubyDataMovementTracker : public ProbeListenerObject
{
 public:
  typedef RubyDataMovementTrackerParams Params;
  RubyDataMovementTracker(const Params &p);
  ~RubyDataMovementTracker() = default;

  void observeWriteback(const SimpleCacheAccessProbeArg &info);
  void regProbeListeners() override;
  /**
   * Print out some statistics
   */
  void print(std::ostream &out) const;
  void setController(AbstractController *_ctrl) { m_controller = _ctrl; }
  void regStats() override;

 private:
  AbstractController *m_controller;

  struct RubyDataMovementTrackerStats : public statistics::Group
  {
    RubyDataMovementTrackerStats(statistics::Group *parent);

    statistics::Scalar numWritebacks;
    std::unordered_map<MachineID, statistics::Scalar *> dataSenderCount;
    std::unordered_map<MachineID, statistics::Histogram *> \
        dataSenderLatencyHistogram;
  } stats;
};

}  // namespace ruby
}  // namespace gem5

#endif  // __MEM_RUBY_STRUCTURES_RUBY_DATA_MOVEMENT_TRACKER_HH__
