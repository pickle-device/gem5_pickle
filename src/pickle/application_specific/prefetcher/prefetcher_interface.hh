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

#ifndef __PREFETCHER_INTERFACE_HH__
#define __PREFETCHER_INTERFACE_HH__

#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "base/statistics.hh"
#include "params/PrefetcherInterface.hh"
#include "pickle/application_specific/pickle_job.hh"
#include "sim/clocked_object.hh"
#include "sim/sim_object.hh"


class c_cerebellum;

namespace gem5
{

enum PacketStatus
{
    SENT = 0,
    ARRIVED
};

class PickleDevice;

class PrefetcherInterface: public ClockedObject
{
    private:
        int64_t prefetch_distance;
        int64_t prefetch_distance_offset_from_software_hint;
        PARAMS(PrefetcherInterface);
    private:
        std::unique_ptr<c_cerebellum> prefetcher;
        std::unordered_map<Addr, std::unique_ptr<uint8_t[]>> packet_data;
        std::unordered_map<Addr, PacketStatus> packet_status;
        bool prefetcher_initialized;
        // check and send prefetch requests for every cycle
        void processPrefetcherOutQueue();
        // check and receive prefetch requests for every cycle
        void processPrefetcherInQueue();
    public:
        PickleDevice* owner;
    public:
        PrefetcherInterface(const PrefetcherInterfaceParams &params);
        ~PrefetcherInterface();
        void startup() override;
    public:
        void setOwner(PickleDevice* engine);
        void clockTick(); // what to do every cycle
        void configure(const PickleJobDescriptor& job);
        void switchOn();
        void switchOff();
        bool isActivated() const { return prefetcher_initialized; }
        uint64_t getPrefetchDistance() const;
    public: // the interface
        bool enqueueWork(const uint64_t& node_id);
        void receivePrefetch(
            const uint64_t& vaddr, std::unique_ptr<uint8_t[]> p
        );
    public:
        uint64_t workCount;
        void regStats() override;
        struct PrefetcherStats : public statistics::Group
        {
            PrefetcherStats(statistics::Group *parent);
            void regStats() override;
            // See the .cc for the description of each stat
            statistics::Scalar numReceivedWork;
            statistics::Scalar numPrefetchesSent;
            statistics::Scalar numPrefetches;
            statistics::Histogram histInQueueLength;
            statistics::Histogram histOutQueueLength;
        } prefetcherStats;
};

}; // namespace gem5

#endif // __PREFETCHER_INTERFACE_HH__
