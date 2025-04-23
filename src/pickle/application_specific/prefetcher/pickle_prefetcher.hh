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

#ifndef __PICKLE_PREFETCHER_HH__
#define __PICKLE_PREFETCHER_HH__

#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "base/statistics.hh"
#include "params/PicklePrefetcher.hh"
#include "pickle/application_specific/pickle_job.hh"
#include "pickle/application_specific/prefetcher/prefetch_generators/all_prefetch_generators.hh"
#include "pickle/application_specific/prefetcher/prefetcher_work_tracker.hh"
#include "pickle/application_specific/prefetcher/work_item.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"
#include "sim/sim_object.hh"

namespace gem5
{

enum PacketStatus
{
    SENT = 0,
    ARRIVED
};

class PickleDevice;

class PicklePrefetcher: public ClockedObject
{
    private:
        int64_t software_hint_prefetch_distance;
        int64_t prefetch_distance_offset_from_software_hint;
        PARAMS(PicklePrefetcher);
        EventFunctionWrapper processInQueueEvent;
        EventFunctionWrapper processOutQueueEvent;
        uint64_t ticks_per_cycle;
        uint64_t num_cores;
        std::shared_ptr<PrefetchGenerator> prefetch_generator;
    private:
        std::unordered_map<Addr, std::unique_ptr<uint8_t[]>> packet_data;
        std::unordered_map<Addr, PacketStatus> packet_status;
        std::unordered_set<Addr> received_packets_to_be_processed;
        // there is a work tracker for each prefetch kernel for each core
        std::vector<std::vector<std::shared_ptr<PrefetcherWorkTracker>>> \
            prefetcher_work_trackers;
        bool prefetcher_initialized;
        // check and send prefetch requests for every cycle
        void processPrefetcherOutQueue();
        // check and receive prefetch requests for every cycle
        void processPrefetcherInQueue();
    public:
        PickleDevice* owner;
    public:
        PicklePrefetcher(const PicklePrefetcherParams &params);
        ~PicklePrefetcher();
        void startup() override;
    public:
        void setOwner(PickleDevice* engine);
        void configure(std::shared_ptr<PickleJobDescriptor> job);
        void switchOn();
        void switchOff();
        bool isActivated() const { return prefetcher_initialized; }
        uint64_t getSoftwareHintPrefetchDistance() const;
        uint64_t getPrefetchDistanceOffsetFromSoftwareHint() const;
        std::shared_ptr<PrefetchGenerator> getPrefetchGenerator() const;
    public: // the interface
        bool enqueueWork(
            const uint64_t workData, const uint64_t prefetchKernelId,
            const uint64_t cpuId
        );
        void receivePrefetch(
            const uint64_t vaddr, std::unique_ptr<uint8_t[]> p
        );
        void scheduleDueToIncomingPrefetch();
        void scheduleDueToNewOutstandingPrefetchRequests();
        PacketPtr zeroCycleLoadWithVAddr(const Addr& vaddr, bool& success);
        PacketPtr zeroCycleLoadWithPAddr(const Addr& paddr, bool& success);
    public: // profile functions
        void profilePrefetchWithUnknownVAddr();
    public:
        uint64_t workCount;
        void regStats() override;
        struct PrefetcherStats : public statistics::Group
        {
            PrefetcherStats(statistics::Group *parent);
            void regStats() override;
            statistics::Scalar numReceivedWork;
            statistics::Scalar numPrefetches;
            statistics::Scalar numUnknownPrefetches;
            statistics::Histogram histInQueueLength;
            statistics::Histogram histOutQueueLength;
        } prefetcherStats;
        struct TaskStats : public statistics::Group
        {
            TaskStats(statistics::Group *parent, const uint64_t cpuId);
            void regStats() override;
            statistics::Scalar taskCount;
            //statistics::Histogram queuedTime;
            statistics::Histogram prefetchLv0Time;
            statistics::Histogram prefetchLv1Time;
            statistics::Histogram prefetchLv2Time;
            statistics::Histogram prefetchLv3Time;
            statistics::Histogram totalPrefetchTime;
            statistics::Histogram timelyPrefetchesDistance;
            statistics::Histogram latePrefetchesDistance;
        };
        std::vector<std::shared_ptr<TaskStats>> taskStats;
        void profileWork(
            std::shared_ptr<WorkItem> work, const uint64_t core_id
        );
        void profileTimelyPrefetch(
            const Tick pf_complete_time, const uint64_t core_id
        );
};

}; // namespace gem5

#endif // __PICKLE_PREFETCHER_HH__
