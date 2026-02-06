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

from m5.objects import *
from m5.params import *
from m5.proxy import *
from m5.SimObject import *


# The backend of a prefetcher, responsible for prefetch request issuance and
# prefetch tracking.
class PrefetchAgent(BasePrefetcher):
    type = "PrefetchAgent"
    cxx_class = "gem5::prefetch::dmp::PrefetchAgent"
    cxx_header = (
        "mem/cache/prefetch/differential_matching_prefetcher/prefetch_agent.hh"
    )

    system = Param.System(Parent.any, "System this prefetcher belongs to")
    clock_domain = Param.ClockDomain(
        Parent.any, "Clock domain for this prefetcher"
    )

    # Prefetch queue
    prefetch_queue = Param.DifferentialMatchingPrefetcherPrefetchQueue(
        "Prefetch queue for this prefetcher"
    )

    # Cache access observation parameters
    on_miss = False
    on_read = True
    on_write = True
    on_data = True
    on_inst = True
    prefetch_on_access = False
    prefetch_on_pf_hit = True
    use_virtual_addresses = False
    page_bytes = "4KiB"
