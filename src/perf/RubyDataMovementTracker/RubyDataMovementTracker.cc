/*
 * Copyright (c) 2024 The Regents of the University of California
 * All rights reserved
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "perf/RubyDataMovementTracker/RubyDataMovementTracker.hh"

#include <cassert>
#include <map>
#include <string>

#include "base/bitfield.hh"
#include "base/cprintf.hh"
#include "base/statistics.hh"
#include "debug/RubyDataMovementTrackerDebug.hh"
#include "debug/RubyDataMovementTrackerStatsDebug.hh"
#include "mem/ruby/common/MachineID.hh"
#include "mem/ruby/slicc_interface/RubySlicc_ComponentMapping.hh"
#include "mem/ruby/system/RubySystem.hh"
#include "sim/probe/probe.hh"

namespace gem5 {

namespace ruby {

RubyDataMovementTracker::RubyDataMovementTracker(const Params& p)
    : ProbeListenerObject(p), m_controller(p.controller), stats(this) {}

RubyDataMovementTracker::RubyDataMovementTrackerStats::
    RubyDataMovementTrackerStats(statistics::Group* parent)
    : statistics::Group(parent, "data_movement_tracker"),
      ADD_STAT(numWritebacks, statistics::units::Count::get(),
               "Number of cache fills observed (non-eviction fill)") {}

void RubyDataMovementTracker::regStats() {
  ProbeListenerObject::regStats();
  RubySystem* ruby_system = m_controller->getRubySystem();
  std::map<std::string, MachineID> sorted_machine_ids;
  for (auto p: ruby_system->machineIDToControllerMap) {
      MachineID machineID = p.first;
      std::string machine_id_str = MachineIDToString(machineID);
      sorted_machine_ids[machine_id_str] = machineID;
  }
  for (auto p : sorted_machine_ids) {
    std::string machine_id_str = p.first;
    MachineID machineID = p.second;
    AbstractController* controller = \
      ruby_system->machineIDToControllerMap[machineID];
    std::string count_stat_desc = \
        csprintf(
          "Number of cache fills observed from %s", controller->name()
        );
    stats.dataSenderCount[machineID] = new statistics::Scalar(
        &stats, machine_id_str.c_str(), statistics::units::Count::get(),
        count_stat_desc.c_str());
    // stats.dataSenderCount[machineID]->flags(statistics::nozero);
    std::string latency_hist_stat_desc =
        csprintf("Latency of receiving data blocks from %s",
                 controller->name());
    stats.dataSenderLatencyHistogram[machineID] = new statistics::Histogram(
        &stats, machine_id_str.c_str(), statistics::units::Tick::get(),
        latency_hist_stat_desc.c_str());
    constexpr uint64_t num_bins = 10;
    stats.dataSenderLatencyHistogram[machineID]->init(num_bins);
  }
}

void RubyDataMovementTracker::observeWriteback(
    const SimpleCacheAccessProbeArg& info) {
  stats.numWritebacks++;
  if (info.machineIDValid) {
    statistics::Scalar* s = stats.dataSenderCount[info.machineID];
    (*s)++;
    stats.dataSenderLatencyHistogram[info.machineID]->sample(info.latency);
  }
  DPRINTF(RubyDataMovementTrackerDebug, "Observed writeback\n");
}

void RubyDataMovementTracker::regProbeListeners() {
  typedef ProbeListenerArg<RubyDataMovementTracker, SimpleCacheAccessProbeArg>
      RubyDataMovementListeners;
  listeners.push_back(new RubyDataMovementListeners(
      this,                                       // object to be invoked
      "DataMovementWriteback",                    // trigger event
      &RubyDataMovementTracker::observeWriteback  // called upon event
      ));
}

void RubyDataMovementTracker::print(std::ostream& out) const {}

}  // namespace ruby
}  // namespace gem5
