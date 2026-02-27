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

from m5.objects.Probe import ProbeListenerObject
from m5.objects.System import System
from m5.params import *
from m5.proxy import *
from m5.SimObject import *


# This SimObject is responsible for tracking the cache blocks above a certain
# level in the cache hierarchy (e.g., L2 and L3).
#
# For example, if it is designated to track the cache blocks within a CCD,
# then it will track the cache blocks in the L1, L2 and LLC caches within that
# CCD.
#
# Similarly, if it is designated to track the cache blocks within a core
# complex, then it will track the cache blocks in the L1, and L2 caches within
# that core complex.
#
# Implementation-wise, this tracker listens to,
# - Demand requests from CPU sequencers.
# - Cache directory at the lowest level (e.g., LLC directory will track all
#   cache blocks of all caches above it in the system, i.e., all L1 and L2
#   caches within that CCD).
# - Cache fills and evictions/invalidation from cache controller at the lowest
#   level. This is necessary because the LLC directory does not neccessarily
#   track all cache blocks in the LLC.
#
# Note that this tracker differentiates the demand requestors and the
# prefetcher requestors by prefetcher IDs.
#
# This tracker produces the following statistics,
# - For each cache block consumed by the CPU, the tracker records the time it
#   was first brought into the cache hierarchy and which requestor brought it
#   in. This is useful for understanding the prefetch usefulness.
# - For every cache block brought into the cache hierarchy by a prefetcher, the
#   tracker records whether that cache block was ever consumed by the CPU
#   before it was evicted. This is useful for understanding the prefetch
#   accuracy.
class RubyCacheBlockTracker(ProbeListenerObject):
    type = "RubyCacheBlockTracker"
    cxx_class = "gem5::ruby::RubyCacheBlockTracker"
    cxx_header = "perf/RubyCacheBlockTracker/RubyCacheBlockTracker.hh"
    cxx_exports = [PyBindMethod("addEventProbe")]

    system = Param.System(Parent.any, "System this is part of")
    ruby_system = Param.RubySystem("RubySystem")

    def addDemandSequencer(self, sequencer):
        if not hasattr(self, "_demand_sequencers"):
            self._demand_sequencers = []
        self._demand_sequencers.append(sequencer)

    # Adding the prefetcher requestors. Different prefetchers have different
    # requestors that issue prefetch requests.
    # - Pickle prefetcher uses a sequencer to issue prefetch requests. Thus, it
    #   uses the requestorID of the sequencer.
    # - DMP prefetcher uses DMP::PrefetchQueue to issue prefetch requests.
    #   Thus, it uses the requestorID of the DMP::PrefetchQueue.
    # - Other prefetchers (e.g., stride prefetcher) use the cache controller's
    #   prefetcher proxy to issue prefetch requests. Thus, they use the
    #   requestorID of the cache controller's prefetcher proxy.
    def addPrefetcherRequestor(self, requestor):
        if not hasattr(self, "_prefetcher_requestors"):
            self._prefetcher_requestors = []
        self._prefetcher_requestors.append(requestor)

    def addCacheController(self, cache_controller):
        if not hasattr(self, "_cache_controllers"):
            self._cache_controllers = []
        self._cache_controllers.append(cache_controller)

    def regProbeListeners(self):
        for sequencer in self._demand_sequencers:
            self.getCCObject().addEventProbe(
                sequencer.getCCObject(), "cpu outgoing data request"
            )
        for cache_controller in self._cache_controllers:
            self.getCCObject().addEventProbe(
                cache_controller.getCCObject(), "Directory entry allocation"
            )
            self.getCCObject().addEventProbe(
                cache_controller.getCCObject(), "Directory entry deallocation"
            )
            self.getCCObject().addEventProbe(
                cache_controller.getCCObject(), "DataMovementWriteback"
            )
            self.getCCObject().addEventProbe(
                cache_controller.getCCObject(),
                "DataMovementWritebackFromEviction",
            )
            self.getCCObject().addEventProbe(
                cache_controller.getCCObject(), "DataMovementEviction"
            )
