# Copyright (c) 2025 The Regents of the University of California
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from m5.objects.ClockedObject import ClockedObject
from m5.params import *
from m5.proxy import *
from m5.SimObject import (
    PyBindMethod,
    SimObject,
)


class PicklePrefetcher(ClockedObject):
    type = "PicklePrefetcher"
    cxx_header = "pickle/application_specific/prefetcher/pickle_prefetcher.hh"
    cxx_class = "gem5::PicklePrefetcher"
    cxx_exports = [PyBindMethod("switchOn"), PyBindMethod("switchOff")]

    software_hint_prefetch_distance = Param.Int(1, "Prefetch distance")
    prefetch_distance_offset_from_software_hint = Param.Int(
        0,
        "Prefetch distance offset from software hint",
    )
    concurrent_work_item_capacity = Param.Int(
        0,
        "Number of wokk items that can be prefetched concurrently",
    )
    num_cores = Param.Int(
        8,
        "Number of cores connected to the cache that this prefetcher is "
        "servicing. In the case of the LLC prefetcher, this is the number of "
        "core in the same CCD.",
    )
    expected_number_of_prefetch_generators = Param.Int(
        1,
        "How many prefetch generators will be used. Helps determining how "
        "many sets of task-related stats to allocate.",
    )

    # Optimization parameters
    prefetch_dropping_distance = Param.Int(
        0,
        "Distance at which prefetches are dropped. "
        "If set to 0, prefetches are never dropped.",
    )
