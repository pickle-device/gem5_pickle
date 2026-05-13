# Copyright (c) 2026 The Regents of The University of California
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from m5.objects.System import System
from m5.params import *
from m5.proxy import *
from m5.SimObject import SimObject


class TrackingAction(Enum):
    vals = [
        "NONE",
        "EXIT_SIM",
    ]


class ProgramProgressTracker(SimObject):
    type = "ProgramProgressTracker"
    cxx_class = "gem5::ProgramProgressTracker"
    cxx_header = "perf/ProgramProgressTracker/ProgramProgressTracker.hh"

    tracker_agents = VectorParam.ProgramProgressTrackerAgent(
        "The ProgramProgressTrackerAgents that will observe instruction "
        "commits and report progress to this tracker."
    )
    tracking_pc = Param.Addr(
        "The program counter (PC) value to track for progress updates. We "
        "count how many times the instruction is executed and report progress."
    )
    tracking_interval = Param.UInt64(
        "The number of instructions between progress updates", default=100000
    )
    action_when_threshold_reached = Param.TrackingAction(
        "The action to take when the tracking pc commit count threshold is "
        "reached.",
        default=TrackingAction("NONE"),
    )
    action_threshold = Param.UInt64(
        "The number of times the tracking pc must be committed before the "
        "action takes place. If this value is 0, the action is never taken.",
        default=0,
    )
