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

#ifndef __PREFETCHER_WORK_ITEM_HH__
#define __PREFETCHER_WORK_ITEM_HH__

#include <array>
#include <unordered_set>

#include "base/types.hh"
#include "sim/cur_tick.hh"

namespace gem5
{

class WorkItem
{
    private:
        uint64_t job_id; // which prefetch generator generated this work
        Addr work_id; // used to identify work from the core
        std::array<std::unordered_set<Addr>, 4> expected_prefetches;
        uint64_t curr_level;
        bool core_worked_on_this_work;
        uint64_t num_indirection_levels;

        // statistics
        // when did this work item was received by prefetcher
        Tick work_received_time;
        // when did the prefetcher send the first prefetch
        Tick prefetch_sent_time;
        // when did the prefetcher receive all prefetches for a certain level
        std::array<Tick, 4> prefetch_received_time;
        // when did the prefetcher finish prefetch all four levels
        Tick work_completed_time;
        // when did the core use this work
        Tick core_use_time;
    private:
        void profileWorkItemReceivedTime();
        void profilePrefetchReceivedTime(const uint64_t level);
        void profileWorkCompletedTime();
    public:
        //void profileFirstPrefetchSentTime();
        void profileCoreUseTime();
    public:
        WorkItem();
        WorkItem(const Addr _work_id);
        void setJobId(const uint64_t _job_id);
        //Tick getQueueTime() const;
        Tick getPrefetchLvTime(const uint64_t lv) const;
        Tick getTotalPrefetchTime() const;
        Tick getCoreUseTime() const;
        void notifyCoreIsWorkingOnThisWork();
        bool hasCoreWorkedOnThisWork() const;
        Tick getPrefetchCompleteTime() const;
        Addr getWorkId() const;
        void addExpectedPrefetch(Addr pf_vaddr, const uint64_t level);
        void removeExpectedPrefetch(Addr pf_vaddr);
        const std::unordered_set<Addr>& getCurrLevelExpectedPrefetches() const;
        void moveToNextLevel();
        bool isDoneWithCurrLevel() const;
        bool isDone() const;
        uint64_t getLevel() const;
        Tick getWorkItemReceiveTime() const;
}; // class WorkItem

}; // namespace gem5

#endif // __PREFETCHER_WORK_ITEM_HH__
