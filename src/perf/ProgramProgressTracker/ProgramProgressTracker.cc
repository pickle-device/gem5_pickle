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

#include "perf/ProgramProgressTracker/ProgramProgressTracker.hh"

#include <chrono>
#include <cstdio>

#include "base/statistics.hh"
#include "base/stats/group.hh"
#include "base/types.hh"
#include "enums/TrackingAction.hh"
#include "params/ProgramProgressTracker.hh"
#include "sim/cur_tick.hh"
#include "sim/sim_exit.hh"
#include "sim/sim_object.hh"

namespace gem5
{

ProgramProgressTracker::ProgramProgressTracker(
  const ProgramProgressTrackerParams &p
) : SimObject(p),
    agents(p.tracker_agents),
    tracking_pc(p.tracking_pc),
    tracking_interval(p.tracking_interval),
    action_when_threshold_reached(
        p.action_when_threshold_reached),
    action_threshold(p.action_threshold),
    pc_encounter_count(0),
    stats(this, tracking_pc, agents.size())
{
    uint64_t i = 0;
    for (auto *agent : agents) {
        agent->setOwner(this);
        agent->setID(i);
        i++;
    }
}

void
ProgramProgressTracker::recordPC(const uint64_t agent_id, const Addr pc)
{
    if (pc == tracking_pc) {
        pc_encounter_count++;
        stats.total_pc_count++;
        (*stats.pc_count_per_core[agent_id])++;
        if (pc_encounter_count % tracking_interval == 0) {
            printProgress();
        }
        if (action_threshold != 0 &&
            pc_encounter_count % action_threshold == 0) {
            if (action_when_threshold_reached == enums::TrackingAction::NONE) {
                return;
            } else if (
                action_when_threshold_reached
                    == enums::TrackingAction::EXIT_SIM
            ) {
                exitSimLoop("ProgramProgressTracker: PC 0x%lx is committed "
                    "%lu times", tracking_pc, pc_encounter_count);
            } else {
                panic("Unknown action");
            }
        }
    }
}

void
ProgramProgressTracker::printProgress() const
{
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    char *time_str = std::ctime(&now_c);
    time_str[std::strlen(time_str) - 1] = '\0'; // Remove the newline character
    printf(
        "%s: Tick %ld: ProgramProgressTracker: PC 0x%lx is committed %lu "
        "times\n",
        time_str, curTick(), tracking_pc, pc_encounter_count
    );
}

ProgramProgressTracker::
ProgramProgressTrackerStats::ProgramProgressTrackerStats(
    statistics::Group *parent, const Addr tracking_pc,
    const uint64_t num_agents
) : statistics::Group(parent),
    ADD_STAT(
        total_pc_count,
        statistics::units::Count::get(),
        csprintf(
            "Total number of times the tracking PC (0x%llx) is committed",
            tracking_pc
        ).c_str()
    )
{
    for (uint64_t i = 0; i < num_agents; i++) {
        pc_count_per_core.push_back(
            new statistics::Scalar(
                this,
                csprintf("tracker_%llu_pc_count", i).c_str(),
                statistics::units::Count::get(),
                csprintf(
                    "Number of times the tracking PC (0x%llx) is committed "
                    "by tracker %llu", tracking_pc, i
                ).c_str()
            )
        );
    }
}

};  // namespace gem5
