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

from m5.objects.ClockedObject import ClockedObject
from m5.params import *
from m5.proxy import *


class PrefetchQueueReplacementPolicy(Enum):
    vals = [
        "DROP_NEW_REQUEST",
        "EVICT_OLDEST_REQUEST",
    ]


class CacheLevel(Enum):
    vals = [
        "L1",
        "L2",
        "L3",
    ]


# The backend of the Differential Matching Prefetcher that handles
# prefetch request issuance and tracking.
class DifferentialMatchingPrefetcherPrefetchQueue(ClockedObject):
    type = "DifferentialMatchingPrefetcherPrefetchQueue"
    cxx_class = "gem5::prefetch::dmp::PrefetchQueue"
    cxx_header = (
        "mem/cache/prefetch/differential_matching_prefetcher/prefetch_queue.hh"
    )

    system = Param.System(Parent.any, "System this prefetcher belongs to")
    clock_domain = Param.ClockDomain(
        Parent.any, "Clock domain for this prefetcher"
    )
    mmu = Param.BaseMMU("The MMU of the associated core")
    cache_level = Param.CacheLevel("Cache level for this prefetch queue")

    # Prefetch queue parameters
    queue_size = Param.Int(64, "Number of entries in the prefetch queue")
    request_propagation_delay = Param.Cycles(
        "Delay for prefetch requests to propagate to prefetch queue"
    )
