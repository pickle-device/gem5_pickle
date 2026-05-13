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

#include "perf/ProgramProgressTracker/ProgramProgressTrackerAgent.hh"

#include "cpu/o3/dyn_inst.hh"
#include "cpu/o3/dyn_inst_ptr.hh"
#include "params/ProgramProgressTrackerAgent.hh"
#include "perf/ProgramProgressTracker/ProgramProgressTracker.hh"
#include "sim/probe/probe.hh"

namespace gem5
{

ProgramProgressTrackerAgent::ProgramProgressTrackerAgent(
  const ProgramProgressTrackerAgentParams &p
) : ProbeListenerObject(p),
    owner(nullptr),
    associated_core(p.associated_core)
{
}

void
ProgramProgressTrackerAgent::setOwner(ProgramProgressTracker *owner)
{
    this->owner = owner;
}

void
ProgramProgressTrackerAgent::setID(uint64_t id)
{
    this->agent_id = id;
}

void
ProgramProgressTrackerAgent::observeInstructionCommit(
  const o3::DynInstPtr &dyn_inst
)
{
    owner->recordPC(agent_id, dyn_inst->pcState().instAddr());
}

void
ProgramProgressTrackerAgent::regProbeListeners()
{
    typedef ProbeListenerArg<ProgramProgressTrackerAgent, o3::DynInstPtr>
        CommitListener;
    listeners.push_back(
        new CommitListener(
            this, // object to be invoked
            "Commit", // trigger event, it's O3CPU's ppCommit
            &ProgramProgressTrackerAgent::observeInstructionCommit // callback
        )
    );
}

}  // namespace gem5
