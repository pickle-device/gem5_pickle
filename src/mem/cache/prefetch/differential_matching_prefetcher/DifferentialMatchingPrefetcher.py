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
from m5.params import *
from m5.proxy import *
from m5.SimObject import *


class DifferentialMatchingPrefetcher(ProbeListenerObject):
    type = "DifferentialMatchingPrefetcher"
    cxx_class = "gem5::prefetch::dmp::DifferentialMatchingPrefetcher"
    cxx_header = "mem/cache/prefetch/differential_matching_prefetcher/differential_matching_prefetcher.hh"

    system = Param.System(Parent.any, "System this prefetcher belongs to")
    l1_controller = Param.RubyController(
        "L1 cache controller associated with this prefetcher"
    )

    # DMP Parameters
    index_queue_size = Param.Unsigned(
        8, "Number of entries in the index queue (IQ)"
    )
    indirection_candidate_scoreboard_num_entries = Param.Unsigned(
        4, "Number of entries in the indirection candidate scoreboard (ICS)"
    )
    indirection_candidate_scoreboard_num_candidates_per_entry = Param.Unsigned(
        16,
        "Number of candidates per entry in the indirection candidate scoreboard"
        " (ICS)",
    )
    sample_window_size = Param.Unsigned(
        64, "Number of accesses in the sample window"
    )
    index_table_num_entries = Param.Unsigned(
        16, "Number of entries in the index table"
    )
    tracked_items_per_index_table_entry = Param.Unsigned(
        32, "Number of tracked items per index table entry"
    )
    target_table_num_entries = Param.Unsigned(
        16, "Number of entries in the target table"
    )
    tracked_items_per_target_table_entry = Param.Unsigned(
        8, "Number of tracked items per target table entry"
    )
    matching_shift_amounts = VectorParam.Int64(
        [-4, -3, -2, -1, 1, 2, 3, 4],
        "Shifting amounts for differential matching. A shift amount of "
        "\alpha means we match a[i] with (b[i] >> \alpha). Negative shift "
        "amounts are also supported, meaning a[i] is matched with "
        "(-b[i] << \alpha).",
    )
    indirect_relation_table_num_entries = Param.Unsigned(
        16, "Number of entries in the indirect relation table (IRT)"
    )
    range_table_num_entries = Param.Unsigned(
        4, "Number of entries in the range table"
    )

    # Patches for fixing some parts of the paper
    ics_deprioritize_on_unsuccessful_matching_patch = Param.Bool(
        True,
        "ICS proposes the PC with the most cache misses over the sample "
        "windows, then the pair of index and target PCs is sent to the "
        "matcher. However, we observe that the ICS keeps proposing the same "
        "PC pair even when the matching is not successful. E.g., in BFS, the "
        "work_queue access PC is kept being matched with the visited access "
        "PC. With this patch, when the proposed target PC is not successful, "
        "we lower the score that target PC the next time, allowing the ICS to "
        "allow other PCs to be proposed. This is done by decreasing the "
        "weight of the unsuccessful target PC in the ICS.",
    )
