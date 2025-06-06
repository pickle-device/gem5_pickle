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

from m5.objects import BaseMMU
from m5.objects.ClockedObject import ClockedObject
from m5.params import *
from m5.proxy import *
from m5.SimObject import (
    PyBindMethod,
    SimObject,
)


class PickleDevice(ClockedObject):
    type = "PickleDevice"
    cxx_header = "pickle/device/pickle_device.hh"
    cxx_class = "gem5::PickleDevice"
    cxx_exports = [PyBindMethod("switchOn"), PyBindMethod("switchOff")]

    system = Param.System(Parent.any, "system object")
    device_id = Param.Int(-1, "DeviceID")
    mmu = Param.BaseMMU("The engine MMU")
    isa = Param.BaseISA("The ISA")
    decoder = Param.InstDecoder("The ISA decoder")
    request_manager = Param.PickleDeviceRequestManager(
        "The request manager to handle requests from the device."
    )
    associated_cores = VectorParam.BaseCPU("Associated cores")
    num_cores = Param.Int(1, "Number of cores")
    request_port = RequestPort(
        "Connected to the engine controller to send requests from the engine."
    )
    uncacheable_snoop_port = VectorResponsePort(
        "port for uncacheable req bypassing Ruby"
    )
    uncacheable_forwarders = VectorParam.TrafficSnooper("UncacheableForwarder")
    is_on = Param.Bool(
        False,
        "Whether the Pickle device is powered on or off. "
        "If False, the device will not respond to any requests.",
    )

    # application specifics
    prefetcher = Param.PicklePrefetcher("Prefetcher")

    # design parameters
    # prefetcher = Param.CerebellumPrefetcher(NULL, "Prefetcher")
    core_to_pickle_latency_in_ticks = Param.Int(
        5000,
        "Latency between the core and the Pickle device",
    )
    ticks_per_cycle = Param.Int(
        250, "Number of picoseconds between 2 device cycles"
    )
    uncacheable_response_queue_capacity = Param.Int(
        0,
        "Number of uncacheable responses that the queue can hold."
        "0 means infinite capacity.",
    )
    response_queue_progress_per_cycle = Param.Int(
        0,
        "Number of responses to be sent per cycle."
        "0 means infinite responses per cycle.",
    )
    coalesce_requests = Param.Bool(
        False,
        "Whether to coalesce requests to the same address into a single "
        "request.",
    )
    coalesce_address_translations = Param.Bool(
        False,
        "Whether to coalesce responses to the same address into a single "
        "response.",
    )
