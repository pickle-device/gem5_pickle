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

#ifndef INCLUDED_DESCRIPTIVEDATAPF_HPP
#define INCLUDED_DESCRIPTIVEDATAPF_HPP

#include <cstdint>
#include <iostream>
#include <set>
#include <tuple>
#include <memory>
#include <unordered_map>
#include <vector>

#include "responseEntry.hpp"

#include "mem/packet.hh"


class descriptiveDataPf
{
  public:
    descriptiveDataPf() = delete;
    descriptiveDataPf(std::string fRange, std::string fData);
    descriptiveDataPf(std::istream &addrIstrm, std::istream& dataIstrm);
    descriptiveDataPf(const std::vector<std::tuple<uint64_t, uint64_t, bool, bool, uint64_t, uint64_t, uint64_t>>& info);
    // Generate multiple response entries in the child range of that of the given entry.
    std::vector<responseEntry> operateRsp(const responseEntry& entry);
    // Generate multiple response entries within the same range as but ahead of the given request entry.
    std::vector<responseEntry> operateReq(const requestEntry& entry);
    bool inTriggerRange(uint64_t addr);
    int getRangeId(uint64_t addr);
    // When the core catch up , increase distance after squash outstanding prefetches.
    bool incDistance(int incr, int max){
        m_distance += incr;
        if (m_distance > max) {
            m_distance = max;
            return false;
        } else if (m_distance < 1) {
            m_distance = 1;
            return false;
        } else {
            return true;
        }
    }
    int distance(){  return m_distance; }

    void printAddrRange(std::ostream& ostrm);
    int testBfs();
    int testPr();
    static std::istringstream addrRangeExBfs();
    static std::istringstream dataExBfs();
    static std::istringstream addrRangeExPr();
    static std::istringstream dataExPr();

  private:
    std::bitset<16> getBitMapOfAddr(uint8_t rangeId, uint64_t addr);
    // return the address at the given offset relative to the given address for the rangeID
    uint64_t getAddrAtOffset(uint8_t rangeId, uint64_t addr, uint64_t offset);
    uint64_t getAddrAtOffset(uint8_t rangeId, uint64_t offset){
        return getAddrAtOffset( rangeId, m_ranges[rangeId].first, offset);
    }
    // Use bitmap to extract list of indices within the cache line
    void getBlockIndices(uint8_t rangeId, std::bitset<16> bitMarks, uint8_t stOffset, std::set<int8_t>& v);
    // Find the address in the destRangeId from the given id and offset from RangeId
    uint64_t getPfAddr(uint8_t rangeId, uint8_t destRangeId, int8_t id, int32_t bOffset);

    // Generate multiple response entries n destRangeId from indices in bIndices in rangeId.
    std::vector<responseEntry> genResponseEntries(uint8_t rangeId, uint8_t destRangeId, std::set<int8_t> bIndices, uint64_t bvaddr, uint64_t triggerAddr, n_PageSize::t_PageSize pageSize, uint8_t gtid, bool metaValid);
    std::vector<responseEntry> genResponseEntries(uint8_t rangeId, uint8_t destRangeId, std::set<int8_t> bIndices, const responseEntry& entry) {
        return genResponseEntries(rangeId, destRangeId, bIndices, entry.bvaddr, entry.triggerAddr, entry.pageSize, entry.gtid, entry.metaValid);
    }
    // Insert a new responseEntry for the destRangeId into the given map
    void insertRspEntry(uint8_t destRangeId, uint64_t triggerAddr, n_PageSize::t_PageSize pageSize, uint8_t gtid, uint64_t pfAddr, std::unordered_map<uint64_t, responseEntry>& res, bool metaValid=0, uint32_t meta=0);
    void insertRspEntry(uint8_t destRangeId, const responseEntry& entry, uint64_t pfAddr, std::unordered_map<uint64_t, responseEntry>& res, bool metaValid=0, uint32_t meta=0) {
        insertRspEntry(destRangeId, entry.triggerAddr, entry.pageSize, entry.gtid, pfAddr, res, metaValid, meta);
    }
    // Create one responseEntry for the given pfAddr with given attributes (usually the same as the parent entry)
    responseEntry createResponseEntry(uint8_t destRangeId, uint64_t offset, uint64_t triggerAddr, n_PageSize::t_PageSize pageSize, uint8_t gtid, bool metaValid=0, uint32_t meta=0);



    // for testing only
    int testSimpleGraph();
    int testNonConsecutiveIndexPattern();
    int testConsecutiveRange();
    // current behavior is to lookup (from data store) the end index in the next cacheline anyway.
    // keep track of total number of possible case and max number of outstanding to find a better way to handling it.
    int testRange2CL();

  public:
    void setData(const uint64_t& vaddr, std::unique_ptr<uint8_t[]> p, size_t size) {
        // step 1: figure out which array we need to update
        //const int rangeId = getRangeId(vaddr);
        int rangeId = -1;
        for (uint64_t i = 0; i < m_ranges.size(); i++) {
            auto p = m_ranges[i];
            uint64_t range_left = (p.first >> 6) << 6;
            uint64_t range_right = (p.second >> 6) << 6;
            if ((vaddr >= range_left) && (vaddr <= range_right))
            {
                rangeId = i;
                break;
            }
        }
        if (rangeId == -1) {
            std::cout << "warn: queried data does not belong to any array vaddr 0x" << std::hex << vaddr << std::dec << std::endl;
            return;
        }
        // step 2: figure out whether we are writing to m_dataI or m_dataP
        const bool indexUsingIndex = m_indexTypes[rangeId];
        // step 3: figure out what is the offset to the array
        const size_t arrayElementSizeInBytes = indexUsingIndex ? 4 : 8;
        uint64_t offsetToArray = (vaddr - m_ranges[rangeId].first) / arrayElementSizeInBytes;
        if (vaddr < m_ranges[rangeId].first)
            offsetToArray = 0;
        uint64_t cacheBlockAddr = (vaddr >> 6) << 6;
        uint64_t offsetToCacheBlock = (vaddr - cacheBlockAddr) / arrayElementSizeInBytes;
        if (vaddr < m_ranges[rangeId].first)
            offsetToCacheBlock = (m_ranges[rangeId].first - vaddr) / arrayElementSizeInBytes;
        // step 4: memcpy data
        if (indexUsingIndex) {
            //std::cout << "set data: rangeId: " << rangeId << ", offset(i32): " << offsetToArray << ", offset to cacheBlock: " << offsetToCacheBlock << std::endl;
            const uint64_t numIters = size / arrayElementSizeInBytes;
            //const uint32_t* dataPtr = dataPkt->getConstPtr<uint32_t>();
            const uint32_t* dataPtr = (const uint32_t*) p.get();
            uint64_t i = offsetToCacheBlock;
            for (; (i < numIters) && (offsetToArray < m_dataI[rangeId].size()); i++, offsetToArray++) {
                m_dataI[rangeId][offsetToArray] = dataPtr[i];
                //std::cout << "setting m_dataI[" << (uint64_t)rangeId << "][" << offsetToArray << "] to 0x" << std::hex << dataPtr[i] << std::dec << std::endl;
            }
        } else {
            //std::cout << "set data: rangeId: " << rangeId << ", offset(i64): " << offsetToArray << ", offset to cacheBlock: " << offsetToCacheBlock << std::endl;
            const uint64_t numIters = size / arrayElementSizeInBytes;
            //const uint64_t* dataPtr = dataPkt->getConstPtr<uint64_t>();
            const uint64_t* dataPtr = (const uint64_t*) p.get();
            uint64_t i = offsetToCacheBlock;
            for (; (i < numIters) && (offsetToArray < m_dataP[rangeId].size()); i++, offsetToArray++) {
                m_dataP[rangeId][offsetToArray] = dataPtr[i];
                //std::cout << "setting m_dataP[" << (uint64_t)rangeId << "][" << offsetToArray << "] to 0x" << std::hex << dataPtr[i] << std::dec << std::endl;
            }
        }
    }


  private:
    enum class rangeType { single, range };
    bool inRange(uint64_t addr, uint8_t rangeId);
    uint64_t getOffset(uint64_t addr) { return addr & m_bOffsetMark; }
    uint64_t getBlockAddr(uint64_t addr) { return addr & ~m_bOffsetMark; }
    // offset is not byte offset. It already take entry size into account.
    uint64_t getData(uint8_t rangeId, uint32_t offset) {
        //std::cout << "get data: rangeId: " << (uint64_t)rangeId << ", offset: " << offset << std::endl;
        if (m_indexTypes[rangeId]) {
//            std::cout << "PM:PM getDataI r " << static_cast<int>(rangeId) << " o " << offset << " v " << std::hex << m_dataI[rangeId][offset] << "\n" << std::dec;
            return m_dataI[rangeId][offset];
        } else {
//            std::cout << "PM:PM getDataP r " << static_cast<int>(rangeId) << " o " << offset << " v " << std::hex << m_dataP[rangeId][offset] << "\n" << std::dec;
            return m_dataP[rangeId][offset];
        }
    }


    // only use in ctor
    void initialize(std::istream &addrIstrm, std::istream& dataIstrm);
    void readAddrRanges(std::istream& istrm);
    void readData(std::istream& istrm);
    std::vector<uint32_t> readDataArray(std::istream &istrm, int num);
    void processData();

  private:
    uint64_t        m_numOutOfRanges = 0;
    const uint64_t  m_bOffsetMark = (((uint64_t)1 << 6) -1);
    const uint8_t   m_triggerRange = 0; // TODO read from file

    // Index to each vector is node index
    //// An index with non-empty vector in m_dataP will have empty vector in m_dataI, and vice versa.
    //// use uint32_t whenevery possible to save memory at a cost of one if statement per access.
    std::vector<std::vector<uint64_t>>        m_dataP; // has one less entry than ranges
    std::vector<std::vector<uint32_t>>        m_dataI; // has one less entry than ranges

    std::vector<std::pair<uint64_t,uint64_t>> m_ranges; // nodes
    std::vector<bool>                         m_indexTypes; // true for index, false for ptr
    std::vector<rangeType>                    m_rangeTypes; // nodes
    std::unordered_map<int8_t, int8_t>        m_edges;  // one fanout
    std::vector<uint8_t>                      m_sizes;  // size of the entry of each range, store 2^ to allow shifting
    std::vector<std::bitset<16>>              m_bitmap; // precompute for ecch size, 4(1), 8(11), 16(1111)

    uint8_t                m_distance = 0;
    const uint8_t          m_type = 0;
};


#endif // !INCLUDED_DESCRIPTIVEDATAPF_HPP
