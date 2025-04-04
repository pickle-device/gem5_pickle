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
 */

#ifndef __PREFETCHER_WORK_TRACKER_HH__
#define __PREFETCHER_WORK_TRACKER_HH__

#include <unordered_set>

#include "mem/packet.hh"
#include "pickle/application_specific/pickle_job.hh"

namespace gem5
{

class PickleDevice;

class WorkItem
{
    private:
        Addr vaddr;
        std::unordered_set<Addr> expected_prefetches;
    public:
        WorkItem() {}
        WorkItem(Addr vaddr) : vaddr(vaddr) {}
        Addr getVAddr() const
        {
            return vaddr;
        }
        void addExpectedPrefetch(Addr addr)
        {
            expected_prefetches.insert(addr);
        }
        const std::unordered_set<Addr>& getExpectedPrefetches() const
        {
            return expected_prefetches;
        }
        void clearExpectedPrefetches()
        {
            expected_prefetches.clear();
        }
        bool hasExpectedPrefetches() const
        {
            return !expected_prefetches.empty();
        }
        void removeExpectedPrefetch(Addr addr)
        {
            expected_prefetches.erase(addr);
        }
        bool isDone() const { return expected_prefetches.empty(); }
        void clear() { expected_prefetches.clear(); }
}; // class WorkItem

class PrefetcherWorkTracker
{
    private:
        PickleDevice* owner;
        std::shared_ptr<PickleJobDescriptor> job_descriptor;
        std::unordered_map<Addr, WorkItem> work_items;
        uint64_t software_hint_distance;
        uint64_t hardware_prefetch_distance;
        uint64_t current_core_work_item;
    public:
        PrefetcherWorkTracker();
        PrefetcherWorkTracker(PickleDevice* owner);
        void setJobDescriptor(
            std::shared_ptr<PickleJobDescriptor> job_descriptor
        );
        void addWorkItem(Addr vaddr);
        void removeWorkItem(Addr vaddr);
        void trackIncomingPrefetch(const Addr pf_vaddr);
};  // class PrefetcherWorkTracker

};  // namespace gem5

#endif // __PREFETCHER_WORK_TRACKER_HH__
