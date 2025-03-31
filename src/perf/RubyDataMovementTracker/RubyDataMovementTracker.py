# Copyright (c) 2024 The Regents of the University of California
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

from m5.objects.Probe import ProbeListenerObject
from m5.objects.System import System
from m5.params import *
from m5.proxy import *
from m5.SimObject import SimObject


class RubyDataMovementTracker(ProbeListenerObject):
    type = "RubyDataMovementTracker"
    cxx_class = "gem5::ruby::RubyDataMovementTracker"
    cxx_header = "perf/RubyDataMovementTracker/RubyDataMovementTracker.hh"

    controller = Param.RubyController("Controller to track data movement")
    ruby_system = Param.RubySystem("RubySystem")
