/*
 * Copyright (c) 2024 Advanced Micro Devices, Inc.
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

#ifndef INCLUDED_CEREBELLUM_HPP
#define INCLUDED_CEREBELLUM_HPP

#include <array>
#include <cstdint>
#include <deque>
#include <iostream>
#include <iterator>
#include <ostream>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "descriptiveDataPf.hpp"
#include "responseEntry.hpp"

typedef uint32_t v_Uint32;
typedef uint64_t v_Uint64;

// ******************* a kernel on the engine MUST use service here to communicate to the cache hierarchy ****************

template <class T>
class boundedQueue
{
  public:
    boundedQueue(int num, bool dropNew); // negative indicate unbounded
    boundedQueue(){ m_num = -1;} // unbounded
    bool isEmpty(){ return m_queue.empty(); }
    bool isFull() { return m_num < 0 ? false : m_queue.size() < m_num;}
    T peekFront() { return m_queue.front().first; }
    T popFront() { T res = m_queue.front().first; m_queue.pop_front(); return res;}
    // return true if successfully push
    bool pushBack(T entry, uint64_t cycle) {
        if ((m_num < 0) || ((m_num > 0) && ((m_queue.size() + 1) < m_num))) {
            m_queue.push_back(std::make_pair(entry, cycle));
            return true;
        } else {
            return false;
        }
    }
    bool pushBackOrMerge(T entry, uint64_t cycle) {
        if ((m_num < 0) || ((m_num > 0) && ((m_queue.size() + 1) < m_num))) {
            m_queue.push_back(make_pair(entry, cycle));
            return true;
        } else {
            return false;
        }
    }

    // return number of entries that were dropped
    int dropOlder(uint32_t x_clk) {
        int num = 0;
        while (!m_queue.empty() && m_queue.front().second < x_clk) {
            m_queue.pop_front();
            ++num;
        }
        return num;
    }
    size_t size() {return m_queue.size();}
    void clear() {m_queue.clear();}

  private:
    int m_num;
    // use deque to allow searching by value
    std::deque<std::pair<T,uint32_t>> m_queue; //data, issued clock cycle
};

template <class T, std::size_t N>
std::ostream& operator<<(std::ostream& o, const std::array<T, N>& arr)
{
    std::copy(arr.cbegin(), arr.cend(), std::ostream_iterator<T>(o, " "));
    return o;
}

template <class T>
class fixedSizeDifferenceAverage
{
public:
    fixedSizeDifferenceAverage(size_t n): size(n) {}
    fixedSizeDifferenceAverage() = delete;

    void reset()    { sum = 0; old = 0; valid = false; m_queue.clear();}
    float average() { return 1.0*sum/m_queue.size(); }
    void insert(T val) {
        // store diff if old value is valid
        if (!valid) {
            valid = true;
            sum = 0;
            m_queue.clear();
        } else {
            T dif = val-old;
            sum += dif;
            m_queue.push_back(dif);
        }
        old = val;

        // maintain fixed size
        if (m_queue.size() > size) {
            sum -= m_queue.front();
            m_queue.pop_front();
        }
    }
    void print() const {
      std::cout << " old " << old << " sum " << sum << " num " << m_queue.size() << "\n";
    }
private:
    std::deque<T> m_queue; // store dif of adjacent values
    float  sum = 0; // keep sum of all the entries (difference) in the queue. Don't compute from queue for efficiency
    T      old = 0;
    bool   valid = false;
    const size_t size;
};

template <class T> // expect T to be computable
class differenceAverage
{
public:
    void reset()    { sum = 0; num = 0; old = 0; valid = false; }
    float average() { return 1.0*sum/num; }
    void insert(T val) {
        if (!valid) {
            valid = true;
            sum = 0;
            num = 0;
        } else {
            sum += (val-old);
            ++num;
        }
        old = val;
    }
    void print() const {
      std::cout << " old " << old << " sum " << sum << " num " << num << "\n";
    }
private:
    bool   valid = false;
    T      old = 0;
    float  sum = 0;
    size_t num = 0;
};



class c_cerebellum_pf_stat
{
    private:
        enum class type {
            hit = 0,
            miss = 1,
            fill = 2,
            onOut = 3,
            offOut = 4,
            evict = 5
        };
    public:
        void regHit(uint64_t addr, uint32_t cycle) {
//            std::cout << "PM:PM regHit " << std::hex << addr << std::dec << " cycle " << cycle << "\n";
            regVal(type::hit,addr,cycle);
        }
        void regMiss(uint64_t addr, uint32_t cycle) {
//            std::cout << "PM:PM regMiss " << std::hex << addr << std::dec << " cycle " << cycle << "\n";
            regVal(type::miss,addr,cycle);
        }
        void regFill(uint64_t addr, uint32_t cycle){
//            std::cout << "PM:PM regFill " << std::hex << addr << std::dec << " cycle " << cycle << "\n";
            regVal(type::fill,addr,cycle);
        }
        void regOnOut(uint64_t addr, uint32_t cycle){
            regVal(type::onOut,addr,cycle);
        }
        void regOffOut(uint64_t addr, uint32_t cycle){
            regVal(type::offOut,addr,cycle);
        }
        void regEvict(uint64_t addr, uint32_t cycle){
//            std::cout << "PM:PM regEvict " << std::hex << addr << std::dec << " cycle " << cycle << "\n";
            regVal(type::evict,addr,cycle);
        }
        void computePfStat() {
            float l_onOffSum   = 0; size_t l_onOffCnt   = 0;
            float l_offFillSum = 0; size_t l_offFillCnt = 0;
            float l_fillHitSum = 0; size_t l_fillHitCnt = 0;
            float l_missOffSum = 0; size_t l_missOffCnt = 0;
            float l_missHitSum = 0; size_t l_missHitCnt = 0;
            float l_onEvictSum = 0; size_t l_onEvictCnt = 0;
            m_onOffMax = 0;
            m_offFillMax = 0;
            m_fillHitMax = 0;
            m_missOffMax = 0;
            m_missHitMax = 0;
            m_onEvictMax = 0;

            for (const auto& i : m_data) {
                const auto& ar = i.second;
                collectDiff(ar, type::offOut, type::onOut , l_onOffSum  , l_onOffCnt  , m_onOffMax);
                collectDiff(ar, type::fill  , type::offOut, l_offFillSum, l_offFillCnt, m_offFillMax);
                collectDiff(ar, type::hit   , type::fill  , l_fillHitSum, l_fillHitCnt, m_fillHitMax); // useful
                collectDiff(ar, type::offOut, type::miss  , l_missOffSum, l_missOffCnt, m_missOffMax); // late
                collectDiff(ar, type::hit   , type::miss  , type::offOut, l_missHitSum, l_missHitCnt, m_missHitMax); // fill and then evict without hit
                collectDiff(ar, type::evict , type::onOut  , type::hit   , l_onEvictSum, l_onEvictCnt, m_onEvictMax); // fill and then evict without hit
            }
            m_onOffAvg     = l_onOffSum   / l_onOffCnt;
            m_offFillAvg   = l_offFillSum / l_offFillCnt;
            m_fillHitAvg   = l_fillHitSum / l_fillHitCnt;
            m_missOffAvg   = l_missOffSum / l_missOffCnt;
            m_missHitAvg   = l_missHitSum / l_missHitCnt;
            m_onEvictAvg = l_onEvictSum / l_onEvictCnt;

            m_onOffCnt     = l_onOffCnt;
            m_offFillCnt   = l_offFillCnt;
            m_fillHitCnt   = l_fillHitCnt;
            m_missOffCnt   = l_missOffCnt;
            m_missHitCnt   = l_missHitCnt;
            m_onEvictCnt = l_onEvictCnt;

            m_data.clear(); // the simulation might resume in the next phase
        }
        float onOffAvg()    {return m_onOffAvg;}
        float offFillAvg()  {return m_offFillAvg;}
        float fillHitAvg()  {return m_fillHitAvg;}
        float missOffAvg()  {return m_missOffAvg;}
        float missHitAvg()  {return m_missHitAvg;}
        float onEvictAvg(){return m_onEvictAvg;}
        float onOffCnt()    {return m_onOffCnt;}
        float offFillCnt()  {return m_offFillCnt;}
        float fillHitCnt()  {return m_fillHitCnt;}
        float missOffCnt()  {return m_missOffCnt;}
        float missHitCnt()  {return m_missHitCnt;}
        float onEvictCnt(){return m_onEvictCnt;}
        float onOffMax()    {return m_onOffMax;}
        float offFillMax()  {return m_offFillMax;}
        float fillHitMax()  {return m_fillHitMax;}
        float missOffMax()  {return m_missOffMax;}
        float missHitMax()  {return m_missHitMax;}
        float onEvictMax(){return m_onEvictMax;}

    private:
        // compute if b > a
        void collectDiff(const std::array<uint32_t,6>& ar, c_cerebellum_pf_stat::type bt, c_cerebellum_pf_stat::type at, float& sum, size_t& cnt, size_t& mx) {
            uint32_t b = ar[static_cast<std::underlying_type<c_cerebellum_pf_stat::type>::type>(bt)];
            uint32_t a = ar[static_cast<std::underlying_type<c_cerebellum_pf_stat::type>::type>(at)];
            if ((b!=0) && (a!=0)) {
                uint32_t diff = b - a;
                if (diff > 0) {
                    sum += b-a;
                    ++cnt;
                }
                mx = std::max<size_t>(mx, diff);
            }
        }
        // b > a and c must be 0 to collect
        void collectDiff(const std::array<uint32_t,6>& ar, c_cerebellum_pf_stat::type bt, c_cerebellum_pf_stat::type at, c_cerebellum_pf_stat::type ct, float& sum, size_t& cnt, size_t& mx) {
            uint32_t b = ar[static_cast<int>(bt)];
            uint32_t a = ar[static_cast<int>(at)];
            uint32_t c = ar[static_cast<int>(ct)];

            if ((b!=0) && (a!=0) && (c==0)) {
                uint32_t diff = b - a;
                if (diff > 0) {
                    sum += b-a;
                    ++cnt;
                }
                mx = std::max<size_t>(mx, diff);
            }
        }
        void regVal(c_cerebellum_pf_stat::type t, uint64_t addr, uint32_t cycle) {
            int i = static_cast<int>(t);
            auto itr = m_data.find(addr);
            if (itr != m_data.end()) {
                if ((itr->second)[i] == 0) { // only register the first time
                    (itr->second)[i] = cycle;
//                    std::cout << "PM:PM regval e " << std::hex << addr << std::dec << " " << m_data[addr] << " " << i << "\n";
                }
            } else {
                std::array<uint32_t,6> ar{{0,0,0,0,0,0}};
                ar[i] = cycle;
                m_data[addr] = ar;
//                std::cout << "PM:PM regval n " << std::hex << addr << std::dec << " " << m_data[addr] << " " << i << "\n";
            }
        }

        std::unordered_map<uint64_t,std::array<uint32_t,6>> m_data;
        size_t m_onOffCnt = 0;
        size_t m_offFillCnt = 0;
        size_t m_fillHitCnt = 0;
        size_t m_missOffCnt = 0;
        size_t m_missHitCnt = 0;
        size_t m_onEvictCnt = 0;
        size_t m_onOffMax = 0;
        size_t m_offFillMax = 0;
        size_t m_fillHitMax = 0;
        size_t m_missOffMax = 0;
        size_t m_missHitMax = 0;
        size_t m_onEvictMax = 0;
        float m_onOffAvg = 0;
        float m_offFillAvg = 0;
        float m_fillHitAvg = 0;
        float m_missOffAvg = 0;
        float m_missHitAvg = 0;
        float m_onEvictAvg = 0;
};

// Represent all hardware for Cerebellum running at core clock.
// These includes general services such as V-P address translation and CAM to track returned data.
// Other specific operations running on the configurable engine are in their respective files.


// CAM to observe and filter return data. This can used by a kernel on the engine, ie., descriptive prefetcher, to attach data to transactions of interest.
// CAM is indexed by physical addresses and mapped to 10 or more bytes. 1st byte is always to indicate the kind of stored data to dispatch to the correct kernel.
// For descriptive prefetcher, this will implement PFHR.

namespace gem5
{
    class PrefetcherInterface;
}; // namespace gem5

class c_cerebellum
{
// TODO: now it work for descriptive prefetcher. generalize it!
// Req queue.  TODO: right now only one queue. How can we serve multiple kernels?
private:
    gem5::PrefetcherInterface* ctrl = nullptr;
public:
    c_cerebellum() {}
    c_cerebellum(gem5::PrefetcherInterface* _ctrl) : ctrl(_ctrl) {}
    void operate(uint64_t clk); // call every core clock
    void captureRequest(uint64_t clk, const uint64_t& workData);
    // Response queue. TODO: right now only one queue. How can we serve multiple kernels?
    // May need multiple queues because they will be more response than request! Can be mimic by taking multiple entry from the queue.
    //void captureResponse(c_K12MemifRequest* x_rsp, uint64_t clk);
    std::vector<uint64_t> captureResponse(uint64_t clk, const gem5::Addr& vaddr, std::unique_ptr<uint8_t[]> p, size_t size);

    void config();
    void configure(const std::vector<std::tuple<uint64_t, uint64_t, bool, bool, uint64_t, uint64_t, uint64_t>>& jobTuples);

    bool hasNextPf();
    //std::tuple<v_Uint64,n_PageSize::t_PageSize,uint8_t> nextPf();
    v_Uint64 nextPf();
    void popNextPf(uint64_t clk);

//// stat API
public:
    // for information only
    size_t maxDistance(){ return m_maxDistance; }
    size_t numSkip()    { return m_numSkip = 0; }
    size_t maxReqQSize(){ return m_maxReqQSize; }
    size_t maxRspQSize(){ return m_maxRspSize; }
    size_t maxOutQSize(){ return m_maxOutQSize;}
    size_t maxOutStandingQSize(){ return m_maxOutStandingQSize;}
    size_t reqQSize(){ return m_reqQ.size(); }
    size_t rspQSize(){ return m_rspQ.size(); }
    size_t outQSize(){ return m_outQ.size();}
    size_t outstandingQSize(){ return m_outstandingEntries.size();}

    // for infomation only
    void computePfStat() {
        m_stat.computePfStat();
    }
    // onOff = how long in outQ
    // offFill = how long prefetch return
    // fillHit = useful prefetch
    // missOff = very late
    // onEvict = way too early and is evicted before use
    // misshit = does not prefetch
    float onOffAvg()    {return m_stat.onOffAvg();}
    float offFillAvg()  {return m_stat.offFillAvg();}
    float fillHitAvg()  {return m_stat.fillHitAvg();}
    float missOffAvg()  {return m_stat.missOffAvg();}
    float missHitAvg()  {return m_stat.missHitAvg();}
    float onEvictAvg(){return m_stat.onEvictAvg();}
    size_t onOffCnt()    {return m_stat.onOffCnt();}
    size_t offFillCnt()  {return m_stat.offFillCnt();}
    size_t fillHitCnt()  {return m_stat.fillHitCnt();}
    size_t missOffCnt()  {return m_stat.missOffCnt();}
    size_t missHitCnt()  {return m_stat.missHitCnt();}
    size_t onEvictCnt(){return m_stat.onEvictCnt();}
    size_t onOffMax()    {return m_stat.onOffMax();}
    size_t offFillMax()  {return m_stat.offFillMax();}
    size_t fillHitMax()  {return m_stat.fillHitMax();}
    size_t missOffMax()  {return m_stat.missOffMax();}
    size_t missHitMax()  {return m_stat.missHitMax();}
    size_t onEvictMax(){return m_stat.onEvictMax();}

    void statHit(uint64_t addr, uint64_t clk){
        if(m_descPf->getRangeId(addr) >= 0)
            m_stat.regHit(addr,clk);
    }
    void statMiss(uint64_t addr, uint64_t clk){
        if(m_descPf->getRangeId(addr) >= 0)
            m_stat.regMiss(addr,clk);
    }
    void statFill(uint64_t addr, uint64_t clk){
        if(m_descPf->getRangeId(addr) >= 0)
            m_stat.regFill(addr,clk);
    }
    void statEvict(uint64_t addr, uint64_t clk){
        if(m_descPf->getRangeId(addr) >= 0)
            m_stat.regEvict(addr,clk);
    }

//// main operations
private:
    void operateAtCerebellumClock(uint64_t clk, std::vector<responseEntry>& prefetchGroups, std::vector<responseEntry>& readGroups, std::vector<responseEntry>& writeGroups);
    void operateReq(uint64_t clk, std::vector<responseEntry>& prefetchGroups, std::vector<responseEntry>& readGroups, std::vector<responseEntry>& writeGroups);
    void operateRsp(uint64_t clk, std::vector<responseEntry>& prefetchGroups, std::vector<responseEntry>& readGroups, std::vector<responseEntry>& writeGroups);

//// hardware queue management
private:
    // TODO: rename *paddr* because it misleads to believe that they are physical addresses.
    void resetOutputLimit() {m_outLimit = m_nOutRequestsPerCycle;}
    void tryRegisterOutstandingEntry(uint64_t clk, responseEntry a, uint64_t bpaddr);
    bool peekOutstangindEntry(uint64_t bpaddr, responseEntry& entry);
    void removeOutstandingEntry(uint64_t bpaddr);
    bool squashOldEntry(requestEntry entry);
    bool isOutQFull() {
        if (m_outQ.size() > m_outQSizeLim)
            return true;
        return false;
    }
    bool isOutStandingQFull() {
        if (m_outstandingEntries.size() > m_outStandingQLim)
            return true;
        else
            return false;
    }

    // for infomation only
    void writeOutstandingEntries(std::ostream& ostrm, const std::string& msg) const {
        if (m_outstandingEntries.empty())
            return;
        ostrm << "PM:PM writeOutstandingEntries " << msg << "\n";
        for (auto const& x : m_outstandingEntries) {
            ostrm << " addr " << std::hex << x.first << std::dec << "\n";
            for (auto const& y : x.second) {
                y.write(ostrm,"");
            }
        }
        ostrm << std::dec;
    }

    // for infomation only
    void collectQStat(uint64_t clk);

    // structures to communicate to the cache hiearchy and kernels
    //// reqQ is simple queue because it arrives and processes in order with no merging.
    //// communicating to kernels and CH is more complex because the prefetches might be merged with older ones and the results can come out of order.
    //// There are 3 structures.
    //// m_outstandingEntries stores all outstanding prefetches. m_rspQ stores result from CH wait to be processed by a kernel. m_outQ stores prefetches to be processed by CH
    //// All genereted responses from a kernel will be stored in m_outstandingEntries. A response will be placed in m_outQ only if it can't be merged with any outstanding entry (see tryRegisterOutstandingEntry).
    //// CH will process entries from m_outQ in order. When the result of a response come back, it will be inserted into m_rspQ. Both m_outQ and m_rspQ contain only info needed by CH.
    //// To process an entry in m_rspQ, the corresponding entry from m_outstandingEntries will be looked up and send to a kernel to process.
    boundedQueue<requestEntry>  m_reqQ;
    // an address can trigger several kerel types or other attribues
    std::unordered_map<uint64_t,std::vector<responseEntry>>           m_outstandingEntries; // bpaddr->entry, cacheline granularity
    boundedQueue<uint64_t>                                            m_rspQ; // to send response to cerebellum
    boundedQueue<std::tuple<v_Uint64,n_PageSize::t_PageSize,uint8_t>> m_outQ; // (bpaddr, pagesize, gtid), to send request to cache hierarchy
//    std::map<uint64_t,std::unordered_set<uint64_t>>                   m_triggerToBvaddrs; // to help remove old value easier
    size_t m_maxReqQSize = 0;
    size_t m_maxRspSize = 0;
    size_t m_maxOutQSize = 0;
    size_t m_maxOutStandingQSize = 0;

    size_t m_maxDistance = 0;

    size_t m_outQSizeLim = 128;
    size_t m_outStandingQLim = 256;

    // number of child responses to enqueue in one cycle.
    // In HW, generation must pause and continue next cycle. In simulation, it is simpler to generate and enqueue all.
    // To mimic multiple cycle enqueuing,new req or res will not be processed for some cycles = min(#gen, m_children)/m_EnqueuePerCycle.
    size_t m_nEnqueuePerCycle = 16;  // enqueue in core clock
    size_t m_nChildren = 1048576; // number of child responses that can be generated per response
    size_t n_waitCycles = 0;

//// for distance adjustment. TODO: should be in desc_pf, but it can't see which cycle the trigger show up.
private:
//    void setupAutoDecrement(float avg);
//    differenceAverage<uint32_t> m_recordLat; // average trigger latency at current distane. it will be transfered to m_prevAvgLat when increasing distance
//    std::stack<float> m_prevAvgLat; // store average latency of previous lower distances
//    float  m_disDecLatTrigger; // if m_monitorLat went above this and come below, dec dist. computed from the top entry of m_prevAvgLat
//    bool   m_triggerArmed; // true when m_monitorLat go above m_disDecLatTrigger
    fixedSizeDifferenceAverage<uint32_t>* m_monitorLat; // current average latency to compare with m_disDecLatTrigger

// issuer
//// issue as prefetch
//// issue as read
//// issue as write
//// other connection to CH

//// Address translation
private:
    // key is the same as responseEntry.bvaddr.
    std::unordered_map<uint32_t,responseEntry> m_cam; //vaddr -> reponseEntry

//// kernels
private:
    void resetAvailKernels() {
        m_availReqKernels = m_numReqKernels;
        m_availRspKernels = m_numRspKernels;
    }
    uint64_t m_slowFactor = 4;
    std::unordered_map<uint32_t,uint32_t> m_availReqKernels; // number of available of each type in this cycle
    std::unordered_map<uint32_t,uint32_t> m_availRspKernels; // number of available of each type in this cycle
    std::unordered_map<uint32_t,uint32_t> m_numReqKernels;   // number of kernel instances to handle request of a type in Cerebellum
    std::unordered_map<uint32_t,uint32_t> m_numRspKernels;   // number of kernel instances to handle reqponse of a type in Cerebellum
    uint8_t m_nOutRequestsPerCycle = 4;
    uint8_t m_outLimit = m_nOutRequestsPerCycle; // number of output from cerebellum allowed per core's clock

    // kernels can't have states, except for stat. If state is needed, associated ith with entry to keep with Cerebellum's CAM.
public:
    descriptiveDataPf* m_descPf;
private:
    c_cerebellum_pf_stat m_stat;
    size_t m_numSkip = 0;

    uint64_t cycle_count = 0;
};



#endif // !INCLUDED_CEREBELLUM_HPP
