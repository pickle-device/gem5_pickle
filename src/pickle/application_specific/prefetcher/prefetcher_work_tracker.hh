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
#include <list>
#include <memory>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "mem/packet.hh"
#include "pickle/application_specific/pickle_job.hh"
#include "pickle/application_specific/prefetcher/prefetch_request.hh"
#include "pickle/application_specific/prefetcher/work_item.hh"

namespace gem5
{

class PicklePrefetcher;
class PrefetchGenerator;
class PrefetcherWorkTrackerCollective;

class PrefetcherWorkTracker
{
    private:
        uint64_t job_id;
        uint64_t core_id;
        bool is_activated;
    public:
        PicklePrefetcher* owner;
        std::shared_ptr<PrefetcherWorkTrackerCollective> collective;
        std::shared_ptr<PickleJobDescriptor> job_descriptor;
    private:
        // This is a map from work address to its WorkItem
        std::unordered_map<Addr, std::shared_ptr<WorkItem>> \
            work_id_to_work_items_map;
        // This is a map from work address to its prefetch complete time
        // This is used when the prefetch task is done before the core uses it
        std::unordered_map<Addr, Tick> work_item_complete_time;
        std::priority_queue<
            std::shared_ptr<WorkItem>,
            std::vector<std::shared_ptr<WorkItem>>,
            WorkItemPointerOrder
        > pending_work_items;
        uint64_t software_hint_distance;
        uint64_t hardware_prefetch_distance;
        std::shared_ptr<PrefetchGenerator> prefetch_generator;
    public:
        PrefetcherWorkTracker();
        PrefetcherWorkTracker(
            PicklePrefetcher* owner,
            std::shared_ptr<PrefetcherWorkTrackerCollective> collective,
            const uint64_t job_id, const uint64_t core_id,
            std::shared_ptr<PickleJobDescriptor> job_descriptor
        );
        uint64_t getJobId() const { return job_id; }
        uint64_t getCoreId() const { return core_id; }
        void addWorkItem(Addr work_data);
        bool hasPendingWorkItem() const;
        std::shared_ptr<WorkItem> peekNextWorkItem() const;
        void popWorkItem();
        void profileWork(std::shared_ptr<WorkItem> work);
        void tryNotifyCoreCurrentWork(const Addr work_id);
        void stopTrackingWork(const Addr work_id);
        friend class PrefetchGenerator;
};  // class PrefetcherWorkTracker

struct PairHasher
{
    std::uint64_t operator()(const std::pair<uint64_t, uint64_t>& p) const {
        return std::hash<uint64_t>{}(p.first) | \
            (std::hash<uint64_t>{}(p.second) << 16);
    }
};

class PrefetcherWorkTrackerCollective
{
    private:
        // The maximum number of active work items
        uint64_t max_active_work_items;
        // The prefetcher that owns this work tracker
        PicklePrefetcher* owner;
    private:
        // job_id, core_id -> prefetcher_work_tracker
        std::unordered_map<
            std::pair<uint64_t, uint64_t>,
            std::shared_ptr<PrefetcherWorkTracker>,
            PairHasher
        > trackers;
        // Active work items
        std::list<std::shared_ptr<WorkItem>> active_work_items;
        // Prefetches addr -> work_items map, used when a prefetch comes back
        std::unordered_map<
            Addr,
            std::vector<std::shared_ptr<WorkItem>>
        > pf_vaddr_to_work_items_map;
        // The prefetches that have not been sent out yet
        std::priority_queue<
            PrefetchRequest, std::vector<PrefetchRequest>, PrefetchRequestOrder
        > outstanding_prefetch_queue;
    protected: // this structure should be only available to ProfileWorkTracker
        // Tracking prefetch completion time
        // job_id, work_id -> prefetch complete time
        std::unordered_map<
            uint64_t,
            std::unordered_map<uint64_t, Tick>
        > pf_complete_time_map;
        // Tracker core time
        std::unordered_map<
            uint64_t,
            std::unordered_map<uint64_t, Tick>
        > core_start_time_map;
    public:
        PrefetcherWorkTrackerCollective();
        PrefetcherWorkTrackerCollective(const uint64_t max_active_work_items);
        void setOwner(PicklePrefetcher* owner);
        void addPrefetcherWorkTracker(
            const uint64_t job_id, const uint64_t core_id,
            std::shared_ptr<PrefetcherWorkTracker> tracker
        );
        std::shared_ptr<PrefetcherWorkTracker> getPrefetcherWorkTracker(
            const uint64_t job_id, const uint64_t core_id
        );
        std::shared_ptr<WorkItem> getAndPopNextWorkItem();
        bool hasOutstandingPrefetchRequest() const;
        PrefetchRequest peekNextPrefetchRequest() const;
        void popPrefetchRequest();
        void processIncomingPrefetch(const Addr pf_vaddr);
        void populateCurrLevelPrefetches(std::shared_ptr<WorkItem> work);
        void replaceActiveWorkItemsUponCompletion();
        void profilePrefetchCompleteTime(
            const uint64_t job_id, const Addr work_id,
            const Tick complete_time
        );
        Tick getCoreStartTime(const uint64_t job_id, const Addr work_id);
        bool hasCoreWorkedOnThisWork(
            const uint64_t job_id, const Addr work_id
        );
        void setCoreStartTime(
            const uint64_t job_id, const Addr work_id, const Tick start_time
        );
        friend class PrefetcherWorkTracker;
}; // class PrefetcherWorkTrackerCollective

};  // namespace gem5

#endif // __PREFETCHER_WORK_TRACKER_HH__
