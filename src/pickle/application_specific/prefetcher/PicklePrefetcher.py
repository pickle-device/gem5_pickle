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


class PrefetchSchedulingPolicy(Enum):
    vals = [
        "EARLIEST_DEADLINE_FIRST_BASED_ON_HINT_ARRIVAL_TIME",
        "FIRST_IN_FIRST_OUT",
    ]


class PicklePrefetcher(ClockedObject):
    type = "PicklePrefetcher"
    cxx_header = "pickle/application_specific/prefetcher/pickle_prefetcher.hh"
    cxx_class = "gem5::PicklePrefetcher"
    cxx_exports = [PyBindMethod("switchOn"), PyBindMethod("switchOff")]

    # Prefetch strategy parameters
    software_hint_prefetch_distance = Param.Int(1, "Prefetch distance")
    prefetch_distance_offset_from_software_hint = Param.Int(
        0,
        "Prefetch distance offset from software hint",
    )
    prefetch_mode = Param.Int(
        1,
        "Choices: "
        "0 - Invalid"
        "1 - Single prefetch item per hint"
        "2 - Bulk prefetch: more than 1 prefetch item per hint",
    )
    bulk_prefetch_chunk_size = Param.Int(
        16384,
        "Only used when bulk prefetch mode is chosen."
        "If this parameter is N, the prefetch hint will be sent per "
        "N work items. For example, if this parameter is 16384 when running "
        "PR workload, a prefetch hint will be sent when the core is working "
        "on item index 0, 16384, 2*16384, etc."
        "This parameter must be non-zero if the prefetch_mode is bulk",
    )
    bulk_prefetch_num_prefetches_per_hint = Param.Int(
        1,
        "Only used when bulk prfetch mode is chosen."
        "If this parameter is N, the prefetcher will generate N prefetch "
        "items per prefetch hint.",
    )
    prefetch_dropping_distance = Param.Int(
        0,
        "Distance at which prefetches are dropped. "
        "If set to 0, prefetches are never dropped.",
    )
    prefetch_scheduling_policy = Param.PrefetchSchedulingPolicy(
        "EARLIEST_DEADLINE_FIRST_BASED_ON_HINT_ARRIVAL_TIME",
        "The scheduling policy used when issuing prefetches.",
    )
    max_requests_per_level = Param.Int(
        0,
        "The maximum number of prefetch requests that can be in-flight per "
        "indirection level. If set to 0, there is no limit.",
    )
    drop_inflight_prefetches = Param.Bool(
        False,
        "Whether to check and drop request at the time of prefetch issue. If "
        "true, the prefetcher will check if the core is too close at the time "
        "of prefetch issue, and drop the prefetch if the core is within the "
        "dropping distance.",
    )

    # Resource parameters
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

    # Prefetcher design choices
    llc_prefetch_agents = VectorParam.LLCPrefetchAgent(
        "The LLC prefetch agent(s) that this prefetcher sends prefetches to.",
    )

    delegate_last_layer_prefetches_to_llc_agents = Param.Bool(
        False,
        "If true, the prefetcher will delegate the last layer prefetches to "
        "LLC agents. Otherwise, the PicklePrefetcher will issue the last layer "
        "prefetches itself.",
    )

    sssp_threshold_optimization_enabled = Param.Bool(
        True,
        "Whether the SSSP threshold optimization is enabled. If true, the "
        "prefetcher will track the `threshold` variable used in SSSP software "
        "to make prefetch decisions.",
    )

    bc_depth_optimization_enabled = Param.Bool(
        False,
        "Whether the BC depth optimization is enabled. If true, the "
        "prefetcher will track the current depth of the BFS traversal in BC "
        "to make prefetch decisions.",
    )
