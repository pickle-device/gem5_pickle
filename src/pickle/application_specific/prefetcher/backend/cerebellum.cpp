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

#include "cerebellum.hpp"

#include <cassert>
#include <cmath>
#include <fstream>
#include <iterator>
#include <queue>
#include <regex>

#include "pickle/application_specific/prefetcher/prefetcher_interface.hh"

// *************************************** Operate on core clock ***************************************************

void c_cerebellum::captureRequest(uint64_t clk, const uint64_t& workData)
{
    requestEntry entry = requestEntry(0, workData, n_PageSize::e_Page4K, 0, 0);
    m_reqQ.pushBack(entry, clk);
}


bool c_cerebellum::squashOldEntry(requestEntry entry)
{
//    // Every entry in outQ which is older (lower addr) than that of the entry will be removed.
//    // The corresponding entry in the outstandingQ will be deleted too.
//    // Other entry in the outstandingQ which are older will also be deleted. As a result, if there is an entry comming back but can't be found in the outstandingQ, they will be ignored.

    std::cout << "squashOldEntry " << std::hex << entry.bvaddr << std::dec << "\n";

    uint64_t addr = entry.bvaddr;

    std::unordered_map<uint64_t,std::vector<responseEntry>> l_outstandingEntries; // bvaddr->entry, cacheline granularity
    for (const auto& e :  m_outstandingEntries) {
        if (((e.second)[0].triggerAddr) > addr)
            l_outstandingEntries[e.first]=e.second;
    }

    bool squashed = (m_outstandingEntries.size() > l_outstandingEntries.size());
    m_outstandingEntries = l_outstandingEntries;


    // drop squashed entry from outQ
    // TODO: order by trigger addr
    size_t ori_size = m_outQ.size();
    boundedQueue<std::tuple<v_Uint64,n_PageSize::t_PageSize,uint8_t>> l_outQ; // (bvaddr, pagesize, gtid), to send request to cache hierarchy
    while (!m_outQ.isEmpty()) {
        std::tuple<v_Uint64,n_PageSize::t_PageSize,uint8_t> e = m_outQ.peekFront();
        auto itr = m_outstandingEntries.find(std::get<0>(e));
        if (itr != m_outstandingEntries.end()) {
            l_outQ.pushBack(e, 0); // TODO: remove cycle which is never used.
        }
        m_outQ.popFront();
    }

    bool outQSquashed = ori_size > l_outQ.size();
    m_outQ = l_outQ;

    return squashed || outQSquashed;
}


std::vector<uint64_t> c_cerebellum::captureResponse(
    uint64_t clk, const gem5::Addr& vaddr, std::unique_ptr<uint8_t[]> p, size_t size)
{
    std::vector<uint64_t> triggerAddrs;
    if (p) {
        //std::cout << "CaptureResponse: vaddr: 0x" << std::hex << vaddr << std::dec << std::endl;
        m_descPf->setData(vaddr, std::move(p), size);
        m_rspQ.pushBack(vaddr, clk);
        for (auto const& entry: m_outstandingEntries[vaddr]) {
            triggerAddrs.push_back(entry.triggerAddr);
        }
    } else {
        // load response contain no data indicating that it faces a problem such as pagefault.
        removeOutstandingEntry(vaddr);
    }
    return triggerAddrs;
}

void c_cerebellum::operate(uint64_t clk)
{
    collectQStat(clk);
    resetOutputLimit(); // reset the number of max output for one core clock TODO


    if (n_waitCycles > 0) {
        // wait until all previously generated responses were enqueued
        n_waitCycles--;
        return;
    }

    // To mimic engine running at slower clock frequency
    //cycle_count++;
    //if (cycle_count % m_slowFactor != 0)
    //    return;

    m_maxDistance = std::max<size_t>(m_maxDistance, m_descPf->distance());

    std::vector<responseEntry> prefetchGroups, readGroups, writeGroups;
    operateAtCerebellumClock(clk, prefetchGroups, readGroups, writeGroups);
    if (prefetchGroups.size() > m_nChildren)
        prefetchGroups.resize(m_nChildren); // discard the excess
    if (prefetchGroups.size() > 0)
        n_waitCycles = static_cast<size_t>(std::ceil(1.0*prefetchGroups.size()/m_nEnqueuePerCycle))-1; // -1 because one batch can be enqueued this cycle

    for (auto& a : prefetchGroups) {
        tryRegisterOutstandingEntry(clk, a, a.bvaddr);
    }
}

bool c_cerebellum::hasNextPf()
{
    if (!m_outQ.isEmpty() && m_outLimit > 0)
        return true;
    return false;
}

v_Uint64 c_cerebellum::nextPf()
{
    std::tuple<v_Uint64,n_PageSize::t_PageSize,uint8_t> front = \
        m_outQ.peekFront();
    return std::get<0>(front);
}

void c_cerebellum::popNextPf(uint64_t clk)
{
    std::tuple<v_Uint64,n_PageSize::t_PageSize,uint8_t> l_pf = m_outQ.peekFront();
    m_stat.regOffOut(std::get<0>(l_pf), clk);
    m_outQ.popFront();
//    std::cout << "PM:PM regOffOut " << std::hex << std::get<0>(l_pf) << std::dec << " clk " << clk << " outQsize " << m_outQ.size() << "\n";
    --m_outLimit;
}

void c_cerebellum::collectQStat(uint64_t clk)
{
    m_maxReqQSize = std::max(m_maxReqQSize, m_reqQ.size());
    m_maxRspSize = std::max(m_maxRspSize, m_rspQ.size());
    m_maxOutQSize = std::max(m_maxOutQSize, m_outQ.size());
    m_maxOutStandingQSize = std::max(m_maxOutStandingQSize, m_outstandingEntries.size());
}

void c_cerebellum::tryRegisterOutstandingEntry(uint64_t clk, responseEntry a, uint64_t bvaddr)
{
    // try to merge
    bool wasMerge = false;
    auto itr = m_outstandingEntries.find(bvaddr);
    if (itr != m_outstandingEntries.end()) {
        for(auto& e : itr->second) {
            if (e.mergeable(a)) {
                e.merge(a);
                wasMerge = true;
                break;
            }
        }
    }

    if (!wasMerge) {
        if (!isOutStandingQFull() && !isOutQFull()) {
            m_outstandingEntries[bvaddr].push_back(a);
            m_outQ.pushBack(std::make_tuple(bvaddr, a.pageSize, a.gtid), clk);
            m_stat.regOnOut(bvaddr, clk);
//        std::cout << "PM:PM regOnOut " << std::hex << bvaddr << std::dec << " clk " << clk << " outQsize " << m_outQ.size() << "\n";
        } else {
            m_numSkip++;
        }
    }
}

bool c_cerebellum::peekOutstangindEntry(uint64_t bvaddr, responseEntry& entry)
{
    auto itr = m_outstandingEntries.find(bvaddr);
    if (itr != m_outstandingEntries.end()) {
        // TODO: support multiple entries ???
        assert(itr->second.size() == 1);
        entry = itr->second[0];
        return true;
    } else {
//        std::cout << "PM:PM peekOutstangindEntry come up empty! for pbaddr " << std::hex << bvaddr << std::dec << "\n";
        return false;
    }
}

void c_cerebellum::removeOutstandingEntry(uint64_t bvaddr)
{
    m_outstandingEntries.erase(bvaddr);
//    writeOutstandingEntries(std::cout,"cerebellum removeOutstangindEntry");
}

// *************************************** Operate on Cerebellum clock (slower than core's) ***************************************************

void c_cerebellum::operateAtCerebellumClock(uint64_t clk
                           , std::vector<responseEntry>& prefetchGroups
                           , std::vector<responseEntry>& readGroups
                           , std::vector<responseEntry>& writeGroups)
{
    resetAvailKernels();
    //if (m_rspQ.size() > 0)
    //    std::cout << "m_rspQ size: " << m_rspQ.size() << std::endl;
    //if (m_reqQ.size() > 0)
    //    std::cout << "m_reqQ size: " << m_reqQ.size() << std::endl;
    //static uint64_t prev_size = 0;
    //if (m_outQ.size() != prev_size)
    //{
    //    std::cout << "m_outQ size: " << m_outQ.size() << std::endl;
    //    prev_size = m_outQ.size();
    //}
    operateReq(clk, prefetchGroups, readGroups, writeGroups); // append results
    operateRsp(clk, prefetchGroups, readGroups, writeGroups); // append results
}

void c_cerebellum::operateRsp(uint64_t clk
                           , std::vector<responseEntry>& prefetchGroups
                           , std::vector<responseEntry>& readGroups
                           , std::vector<responseEntry>& writeGroups)
{
    while (!m_rspQ.isEmpty()) {
        uint64_t bvaddr = m_rspQ.popFront();
        responseEntry entry;
        if (peekOutstangindEntry(bvaddr, entry)) {
            // possible that return is not in response to cerebellum but the core or it was squashed due to lateness
            if (m_availRspKernels[entry.type] > 0) {
                m_availRspKernels[entry.type] = m_availRspKernels[entry.type] - 1;
                //std::cout << "PM:PM cb operateRsp addr " << std::hex << entry.bvaddr << std::dec << " clk " << clk << " " << " idx " <<  static_cast<int>(entry.curRangeIdx) << "\n";
                // try to avoid polymophism for speed reason
                switch (entry.type) {
                    case 0:
                        //std::cout << "c_cerebellum::operateRsp bvaddr: 0x" << std::hex << bvaddr << std::dec << std::endl;
                        std::vector<responseEntry> grp = m_descPf->operateRsp(entry);
                        prefetchGroups.insert(prefetchGroups.end(),std::make_move_iterator(grp.begin()),std::make_move_iterator(grp.end()));
                        removeOutstandingEntry(bvaddr);
                    break;
                }
            } else {
                // TODO: should skip to next type?
                break;
            }
        }
    }
}

void c_cerebellum::operateReq(uint64_t clk, std::vector<responseEntry>& prefetchGroups, std::vector<responseEntry>& readGroups, std::vector<responseEntry>& writeGroups)
{
    uint32_t type = 0; // TODO

    while (!m_reqQ.isEmpty()) {
        if (m_availReqKernels[type] > 0) {
            m_availReqKernels[type] = m_availReqKernels[type] - 1;
            requestEntry entry = m_reqQ.popFront();
            //std::cout << "****************** Processing vaddr: 0x" << std::hex << entry.bvaddr << std::dec << std::endl;
//            std::cout << "PM:PM cb operateReq addr " << std::hex << entry.bvaddr << std::dec << " clk " << clk << "\n";
            // try to avoid polymophism for speed reason
            switch (entry.type) {
                case 0:
                    std::vector<responseEntry> grp = m_descPf->operateReq(entry);
                    prefetchGroups.insert(prefetchGroups.end(),std::make_move_iterator(grp.begin()),std::make_move_iterator(grp.end()));
                break;
            }
        } else {
            // TODO: should skip to next type?
            break;
        }
    }
}

// *************************************** Helper functions (untimed) ***************************************************

void c_cerebellum::config()
{
    if (0) {
        std::istringstream testRange = descriptiveDataPf::addrRangeExBfs();
        std::istringstream testData = descriptiveDataPf::dataExBfs();
        m_descPf = new descriptiveDataPf(testRange, testData);
        m_descPf->testBfs();
        std::cout << "PM:PM finish testing\n";
        exit(1);
    } else if (0) {
        std::istringstream testRange = descriptiveDataPf::addrRangeExPr();
        std::istringstream testData  = descriptiveDataPf::dataExPr();
        m_descPf = new descriptiveDataPf(testRange, testData);
        m_descPf->testPr();
        std::cout << "PM:PM finish testing\n";
        exit(1);
    } else {
        std::cout << "PM:PM create descriptiveDataPf\n";
        m_descPf = new descriptiveDataPf("range_addr.txt", "range_data.txt");
        m_descPf->printAddrRange(std::cout);
    }
}

void c_cerebellum::configure(const std::vector<std::tuple<uint64_t, uint64_t, bool, bool, uint64_t, uint64_t, uint64_t>>& jobTuples)
{
    m_slowFactor = 1;
    m_numReqKernels[0] = 32;
    m_numRspKernels[0] = 32;
    m_outQSizeLim      = 64000;
    m_outStandingQLim  = 64000;
    m_nEnqueuePerCycle = 32;
    m_nOutRequestsPerCycle = 32;
    m_nChildren        = 32;
    m_descPf = new descriptiveDataPf(jobTuples);
}

