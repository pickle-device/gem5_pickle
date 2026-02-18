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

#include "base/types.hh"
#include "params/ProgramProgressTracker.hh"
#include "sim/cur_tick.hh"
#include "sim/sim_object.hh"

namespace gem5
{

ProgramProgressTracker::ProgramProgressTracker(
  const ProgramProgressTrackerParams &p
) : SimObject(p),
    agents(p.tracker_agents),
    tracking_pc(p.tracking_pc),
    tracking_interval(p.tracking_interval),
    pc_encounter_count(0)
{
    for (auto *agent : agents) {
        agent->setOwner(this);
    }
}

void
ProgramProgressTracker::recordPC(const Addr pc)
{
    if (pc == tracking_pc) {
        pc_encounter_count++;
        if (pc_encounter_count % tracking_interval == 0) {
            printProgress();
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

};  // namespace gem5
