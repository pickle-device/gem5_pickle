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

#include <array>
#include <memory>
#include <queue>
#include <unordered_set>
#include <vector>

#include "mem/packet.hh"
#include "pickle/application_specific/pickle_job.hh"
#include "pickle/application_specific/prefetcher/work_item.hh"

namespace gem5
{

class PrefetcherInterface;
class PrefetchGenerator;

class PrefetcherWorkTracker
{
    private:
        uint64_t id;
        bool is_activated;
    public:
        PrefetcherInterface* owner;
        std::shared_ptr<PickleJobDescriptor> job_descriptor;
    private:
        // This is a map from work address to its WorkItem
        std::unordered_map<Addr, std::shared_ptr<WorkItem>> \
            work_vaddr_to_work_items_map;
        // This is a map from prefetch addresses induced by multiple WorkItem
        std::unordered_map<Addr, std::vector<std::shared_ptr<WorkItem>>> \
            pf_vaddr_to_work_items_map;
        // This is a map from work address to its prefetch complete time
        // This is used when the prefetch task is done before the core uses it
        std::unordered_map<Addr, Tick> pf_complete_time;
        std::queue<Addr> outstanding_prefetches;
        uint64_t software_hint_distance;
        uint64_t hardware_prefetch_distance;
        uint64_t prefetch_distance;
        uint64_t current_core_work_item;
        std::string prefetch_generator_mode;
        std::shared_ptr<PrefetchGenerator> prefetch_generator;
    public:
        PrefetcherWorkTracker();
        PrefetcherWorkTracker(
            PrefetcherInterface* owner, const uint64_t _id,
            std::string prefetch_generator_mode
        );
        void setJobDescriptor(
            std::shared_ptr<PickleJobDescriptor> job_descriptor
        );
        void addWorkItem(Addr vaddr);
        void processIncomingPrefetch(const Addr pf_vaddr);
        void populateCurrLevelPrefetches(std::shared_ptr<WorkItem> work);
        bool hasOutstandingPrefetch() const;
        Addr peekNextPrefetch() const;
        void popPrefetch();
        void profileWork(std::shared_ptr<WorkItem> work);
        void notifyCoreCurrentWork(const Addr work_vaddr);
        friend class PrefetchGenerator;
};  // class PrefetcherWorkTracker

};  // namespace gem5

#endif // __PREFETCHER_WORK_TRACKER_HH__
