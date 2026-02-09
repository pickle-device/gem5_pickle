# Copyright (c) 2026 The Regents of the University of California.
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


class BFSGen(ClockedObject):
    type = "BFSGen"
    cxx_header = "cpu/testers/graph_gen/bfs_gen.hh"
    cxx_class = "gem5::BFSGen"

    system = Param.System(Parent.any, "System object")

    port = RequestPort("Outgoing port")

    cache_block_size = Param.Unsigned(64, "Cache block size in bytes")

    # Debugging parameters
    progress_tracking_interval = Param.Unsigned(
        100000, "Interval for tracking BFS progress in number of responses"
    )

    # Graph configuration parameters
    graph_file = Param.String("Input graph file in adjacency list format")
    is_directed = Param.Bool(
        False, "Whether the input graph is directed or undirected"
    )
    source_vertex = Param.Unsigned("Source vertex for BFS traversal")
    num_visitor_threads = Param.Unsigned(1, "Number of visitor threads")
    max_num_responses = Param.Unsigned(
        0, "Maximum number of responses to process (0 for no limit)"
    )

    work_queue_start_vaddr = Param.Addr(
        0x1000_0000, "Virtual address where the work queue starts"
    )
    work_queue_element_size = Param.Unsigned(
        4, "Size of each work queue element in bytes"
    )
    work_queue_access_pc = Param.Addr(
        0x120, "PC address used for work queue accesses"
    )

    neighbor_ptr_start_vaddr = Param.Addr(
        0x2000_0000, "Virtual address where the neighbor pointer array starts"
    )
    neighbor_ptr_element_size = Param.Unsigned(
        8, "Size of each neighbor pointer element in bytes"
    )
    neighbor_ptr_access_pc = Param.Addr(
        0x140, "PC address used for neighbor pointer accesses"
    )

    neighbor_list_start_vaddr = Param.Addr(
        0x3000_0000, "Virtual address where the neighbor list starts"
    )
    neighbor_list_element_size = Param.Unsigned(
        4, "Size of each neighbor list element in bytes"
    )
    neighbor_list_access_pc = Param.Addr(
        0x160, "PC address used for neighbor list accesses"
    )

    visited_list_start_vaddr = Param.Addr(
        0x4000_0000, "Virtual address where the visited list starts"
    )
    visited_list_element_size = Param.Unsigned(
        4, "Size of each visited list element in bytes"
    )
    visited_list_access_pc = Param.Addr(
        0x180, "PC address used for visited list accesses"
    )
