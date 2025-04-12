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

#include "pickle/application_specific/prefetcher/work_item.hh"

namespace gem5
{

WorkItem::WorkItem()
  : curr_step(0), work_received_time(0), work_completed_time(0),
    core_use_time(0)
{
    prefetch_received_time.fill(0);
}

WorkItem::WorkItem(const Addr _work_vaddr)
  : work_vaddr(_work_vaddr), curr_step(0), work_received_time(0),
    work_completed_time(0), core_use_time(0)
{
  profileWorkItemReceivedTime();
  prefetch_received_time.fill(0);
}

void
WorkItem::profileWorkItemReceivedTime()
{
    work_received_time = curTick();
}

void
WorkItem::profilePrefetchReceivedTime(const uint64_t level)
{
    prefetch_received_time[level] = curTick();
}

void
WorkItem::profileWorkCompletedTime()
{
    work_completed_time = curTick();
}

//void
//WorkItem::profileFirstPrefetchSentTime()
//{
//    prefetch_sent_time = curTick();
//}

void
WorkItem::profileCoreUseTime()
{
    core_use_time = curTick();
}

//Tick
//WorkItem::getQueueTime() const
//{
//    return prefetch_sent_time - work_received_time;
//}

Tick
WorkItem::getPrefetchLvTime(const uint64_t lv) const
{
    if (lv == 0) {
        //return prefetch_received_time[0] - prefetch_sent_time;
        return prefetch_received_time[0] - work_received_time;
    } else {
        return prefetch_received_time[lv] - \
            prefetch_received_time[lv - 1];
    }
}

Tick
WorkItem::getTotalPrefetchTime() const
{
    //return work_completed_time - prefetch_sent_time;
    return work_completed_time - work_received_time;
}

Tick
WorkItem::getCoreUseTime() const
{
    return core_use_time;
}

void
WorkItem::notifyCoreIsWorkingOnThisWork()
{
    core_worked_on_this_work = true;
    profileCoreUseTime();
}

bool
WorkItem::hasCoreWorkedOnThisWork() const
{
    return core_worked_on_this_work;
}

Tick
WorkItem::getPrefetchCompleteTime() const
{
    return work_completed_time;
}

Addr
WorkItem::getWorkVAddr() const
{
    return work_vaddr;
}

void
WorkItem::addExpectedPrefetch(Addr pf_vaddr, const uint64_t step)
{
    expected_prefetches[step].insert(pf_vaddr);
}

void
WorkItem::removeExpectedPrefetch(Addr pf_vaddr)
{
    std::unordered_set<Addr>& curr_set = \
        expected_prefetches[curr_step];
    auto it = curr_set.find(pf_vaddr);
    if (it != curr_set.end()) {
        curr_set.erase(it);
    }
    if (curr_set.empty()) {
        profilePrefetchReceivedTime(curr_step);
        if (curr_step == 3) {
            profileWorkCompletedTime();
        }
    }
}

const std::unordered_set<Addr>&
WorkItem::getCurrStepExpectedPrefetches() const
{
    return expected_prefetches[curr_step];
}

void
WorkItem::moveToNextStep()
{
    curr_step += 1;
}

bool
WorkItem::isDoneWithCurrStep() const
{
    return expected_prefetches[curr_step].empty();
}

bool
WorkItem::isDone() const
{
    return curr_step == 4;
}

uint64_t
WorkItem::getStep() const
{
    return curr_step;
}

}; // namespace gem5
