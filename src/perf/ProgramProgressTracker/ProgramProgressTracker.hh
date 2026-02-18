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

#ifndef __PERF_PROGRAM_PROGRESS_TRACKER_HH__
#define __PERF_PROGRAM_PROGRESS_TRACKER_HH__

#include <vector>

#include "base/types.hh"
#include "params/ProgramProgressTracker.hh"
#include "perf/ProgramProgressTracker/ProgramProgressTrackerAgent.hh"
#include "sim/probe/probe.hh"

namespace gem5
{

class ProgramProgressTracker : public SimObject
{
  private:
    std::vector<ProgramProgressTrackerAgent*> agents;
    Addr tracking_pc;
    uint64_t tracking_interval;

    uint64_t pc_encounter_count;

  public:
    typedef ProgramProgressTrackerParams Params;
    ProgramProgressTracker(const ProgramProgressTrackerParams &p);
    ~ProgramProgressTracker() = default;

    void recordPC(const Addr pc);
    void printProgress() const;
};

}  // namespace gem5

#endif // __PERF_PROGRAM_PROGRESS_TRACKER_HH__
