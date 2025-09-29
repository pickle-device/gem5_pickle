/*
 * Copyright (c) 2025 The Regents of the University of California
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 **/

#ifndef __THREAD_MONITOR_HH__
#define __THREAD_MONITOR_HH__

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "base/types.hh"
#include "sim/cur_tick.hh"

namespace gem5 {

namespace pickle {

class ThreadMonitor
{
 public:
  ThreadMonitor();
  ~ThreadMonitor();
  std::unordered_map<uint64_t, uint64_t> core_thread_map;
  void recordThreadStart(uint64_t thread_id, Tick start_tick);
  void recordThreadEnd(uint64_t thread_id, Tick end_tick);
  // Return the map of thread IDs to their respective vectors of run
  // durations.
  std::unordered_map<uint64_t, std::vector<Tick>> getThreadRunDuration();

 private:
  std::unordered_map<uint64_t, std::vector<Tick>> thread_start_ticks;
  std::unordered_map<uint64_t, std::vector<Tick>> thread_end_ticks;
};  // class ThreadMonitor

};  // namespace pickle

};  // namespace gem5

#endif
