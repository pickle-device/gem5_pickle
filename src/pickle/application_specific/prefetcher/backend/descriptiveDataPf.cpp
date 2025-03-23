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

#include "descriptiveDataPf.hpp"

#include <algorithm> // for max
#include <cmath>
#include <fstream>
#include <regex>


std::vector<responseEntry> descriptiveDataPf::operateReq(const requestEntry& entry)
{
//    std::cout << "PM:PM operateReq bvaddr " << std::hex << entry.bvaddr << std::dec << "\n";
    std::vector<responseEntry> res;

    uint64_t triggerAddr = entry.bvaddr;
    uint64_t m_triggerRange = getRangeId(triggerAddr);
    if (m_rangeTypes[m_triggerRange] == rangeType::single) {
        uint64_t pfAddr        = getAddrAtOffset(m_triggerRange, triggerAddr, m_distance);
        responseEntry newEntry = createResponseEntry(m_triggerRange, pfAddr, pfAddr, entry.pageSize, entry.gtid);
        res.push_back(newEntry);
//        std::cout << "PM:PM operateReq triggerAddr " << std::hex << triggerAddr << " " << static_cast<int>(m_triggerRange) << " pfaddr " << pfAddr << " rip " << entry.rip << std::dec << "\n";
    } else { // range type
        uint64_t begPfAddr = getAddrAtOffset(m_triggerRange, triggerAddr, m_distance);
        uint64_t endPfAddr = getAddrAtOffset(m_triggerRange, triggerAddr, m_distance+1);

//        std::cout << "PM:PM triggerAddr " << std::hex << triggerAddr << " " << static_cast<int>(m_distance) << " beg " << begPfAddr  << " end " << endPfAddr << std::dec << "\n";

        if (getBlockAddr(begPfAddr ) != getBlockAddr(endPfAddr )) { // span two cache lines
            responseEntry newEntry1 = createResponseEntry(m_triggerRange, begPfAddr, begPfAddr, entry.pageSize, entry.gtid, false, 0);
            res.push_back(newEntry1);
            uint32_t bOffset = ( begPfAddr - m_ranges[m_triggerRange].first ) >> m_sizes[m_triggerRange];
            responseEntry newEntry2 = createResponseEntry(m_triggerRange, endPfAddr, begPfAddr, entry.pageSize, entry.gtid, true, bOffset);
            res.push_back(newEntry2);
        } else {
            std::unordered_map<uint64_t, responseEntry> resMap;
            for (uint64_t pfAddr = begPfAddr; pfAddr < endPfAddr; pfAddr += ((uint64_t)1 << m_sizes[m_triggerRange])) {
                insertRspEntry(m_triggerRange, begPfAddr, entry.pageSize, entry.gtid, pfAddr, resMap); // append result to res
            }
            std::transform(resMap.begin(), resMap.end(), std::back_inserter(res), [](std::pair<uint64_t,responseEntry> element){return element.second;});
        }
    }

    //std::cout << "Request vaddr = 0x" <<std::hex << entry.bvaddr << std::dec << " generated " << res.size() << " requests" << std::endl;

    return res;
}


// TODO: Do I need to store state to work in next cycle due to output port limitation?
std::vector<responseEntry> descriptiveDataPf::operateRsp(const responseEntry& entry)
{
    std::vector<responseEntry> v;

    // get source info
    uint8_t rangeId = entry.curRangeIdx;

    //uint64_t triggerAddr = entry.triggerAddr;
    //std::cout << "PM:PM operateRsp tad " << std::hex << triggerAddr  << " bva " << entry.bvaddr << std::dec << "\n";

    // get dest info
    int8_t destRangeId  = m_edges.at(rangeId);
    if (destRangeId < 0)
        return v;

    std::set<int8_t> bIndices;
    std::bitset<16> bits = entry.bitMarks;
    uint8_t stOffset = 0;
    while (!bits.none()) {
        getBlockIndices(rangeId, bits, stOffset, bIndices); // use bitmap to extract list of indices from response cacheline

        // remove leftmost batch of 1's
        while ((bits.test(0) == 0)) {
            bits >>= 1;
            stOffset += 4; // each bit is for 4 bytes
        }
        while ((bits.test(0) == 1)) {
            bits >>= 1;
            stOffset += 4; // each bit is for 4 bytes
        }
//        std::cout << "PM:PM bits " << bits  << "\n";
    }

    return genResponseEntries(rangeId, destRangeId, bIndices, entry.bvaddr, entry.triggerAddr, entry.pageSize, entry.gtid, entry.metaValid);
}

std::vector<responseEntry> descriptiveDataPf::genResponseEntries(uint8_t rangeId, uint8_t destRangeId, std::set<int8_t> bIndices, uint64_t bvaddr, uint64_t triggerAddr, n_PageSize::t_PageSize pageSize, uint8_t gtid, bool metaValid)
{
    // each responseEntry is for one cache line
    std::unordered_map<uint64_t, responseEntry> res;

    // bOffset can be negative because bvaddr can be smaller than the range begining
    int32_t bOffset = ( bvaddr - m_ranges[rangeId].first ) >> m_sizes[rangeId];
//    std::cout << "PM:PM bO " << bOffset << std::hex << " rb " << m_ranges[rangeId].first << "\n" << std::dec;

    if (m_rangeTypes[rangeId] == rangeType::single) {
        for (auto id : bIndices) { // index in rangeId
            uint64_t pfAddr = getPfAddr(rangeId, destRangeId, id, bOffset);
            //std::cout << "descriptiveDataPf::genResponseEntries Single bIndices_id=" << (uint64_t)id << std::endl;
            insertRspEntry(destRangeId, triggerAddr, pageSize, gtid, pfAddr, res); // append result to res
            //std::cout << "descriptiveDataPf::genResponseEntries bIndices_id.pfAddr=" << pfAddr << std::endl;
        }
    } else { // range type
        // TODO: cover adjacent range and multile adjacent ranges
        if (metaValid)
            bIndices.insert(-1);   // TODO should use meta to be hardware correct.

        for (auto id : bIndices) { // index in rangeId
            //std::cout << "descriptiveDataPf::genResponseEntries Range bIndices_id=" << (uint64_t)id << std::endl;
            if (((id+1) << m_sizes[rangeId]) >= 64) { // the end of this range is in another cache line
                uint64_t pfAddr = bvaddr + 64; // next cacheline
                insertRspEntry(rangeId, triggerAddr, pageSize, gtid, pfAddr, res, true, id + bOffset); // append result to res
                //std::cout << "descriptiveDataPf::genResponseEntries bIndices_id.pfAddr=" << pfAddr << ", bvaddr = " << bvaddr << std::endl;
            } else {
                uint64_t begPfAddr = getPfAddr(rangeId, destRangeId, id, bOffset);
                uint64_t endPfAddr = getPfAddr(rangeId, destRangeId, id+1, bOffset);

                for (uint64_t pfAddr = begPfAddr; pfAddr < endPfAddr; pfAddr += ((uint64_t)1 << m_sizes[destRangeId])) {
                    insertRspEntry(destRangeId, triggerAddr, pageSize, gtid, pfAddr, res); // append result to res
                    //std::cout << "descriptiveDataPf::genResponseEntries bIndices_id.pfAddr=" << pfAddr << ", bvaddr = " << bvaddr << std::endl;
                }
            }
        }
//        std::cout << "PM:PM range out size " << res.size() << "\n";
    }

    // TODO: dropping late prefetch tread. Do I need some special structure to detect late one easier?

    std::vector<responseEntry> v;
    std::transform(res.begin(), res.end(), std::back_inserter(v), [](std::pair<uint64_t,responseEntry> element){return element.second;});
    //std::cout << "descriptiveDataPf::genResponseEntries [" << (int64_t)rangeId << "]->[" <<(int64_t)destRangeId << "] bvaddr 0x" <<std::hex<< bvaddr << " triggerAddr 0x" << triggerAddr 
    //          << std::dec << " generated " << v.size() << " responses" << std::endl;
    return v;
}

uint64_t descriptiveDataPf::getPfAddr(uint8_t rangeId, uint8_t destRangeId, int8_t id, int32_t bOffset)
{
    uint64_t pfAddr = 0xdeadbeef;
    uint32_t offset = id + bOffset;
    if (m_indexTypes[rangeId]) {
        pfAddr = getAddrAtOffset(destRangeId, getData(rangeId, offset));
        // std::cout << "PM:PM idx id " << static_cast<int>(id) << " o " << offset << std::hex << " pfA " << pfAddr << "\n";
    } else { // entry is a pointer
        pfAddr = getData(rangeId, offset);
        // std::cout << "PM:PM ptr id " << static_cast<int>(id) << " o " << offset << std::hex << " pfA " << pfAddr << "\n";
    }
    return pfAddr;
}

void descriptiveDataPf::insertRspEntry(uint8_t destRangeId, uint64_t triggerAddr, n_PageSize::t_PageSize pageSize, uint8_t gtid, uint64_t pfAddr, std::unordered_map<uint64_t, responseEntry>& res, bool metaValid, uint32_t meta)
{
    responseEntry newEntry = createResponseEntry(destRangeId, pfAddr, triggerAddr, pageSize, gtid, metaValid, meta);
    //std::cout << "insertRspEntry destRangeId=" << (uint64_t)destRangeId << " triggerAddr: 0x" << std::hex << triggerAddr << " pfAddr: 0x" << pfAddr << std::dec << std::endl;
    auto itr = res.find(newEntry.bvaddr); // TODO: only check addr enough?
    if (itr != res.end())
        newEntry.merge(itr->second);
    res[newEntry.bvaddr] = newEntry;
}

responseEntry descriptiveDataPf::createResponseEntry(uint8_t destRangeId, uint64_t pfAddr, uint64_t triggerAddr, n_PageSize::t_PageSize pageSize, uint8_t gtid, bool metaValid, uint32_t meta)
{
    std::bitset<16> bits = getBitMapOfAddr(destRangeId, pfAddr);
    uint64_t pfBAddr     = getBlockAddr(pfAddr);
    responseEntry entry(m_type, pfBAddr, destRangeId, triggerAddr, bits, pageSize, gtid, metaValid, meta);
    if (metaValid)  // when creating entry with metaValid, bitMarks must be 0. However, an entry can have metaValid true and non-zero bitMarks after merging.
        entry.bitMarks = 0;
    return entry;
}

void descriptiveDataPf::getBlockIndices(uint8_t rangeId, std::bitset<16> bitMarks, uint8_t stOffset, std::set<int8_t>& v)
{
//    std::cout << "PM:PM getBlockIndices r " << static_cast<int>(rangeId) << std::hex << " " << bitMarks << std::dec << "\n";

    // index of first 1
    uint8_t oneOffset = stOffset;
    while ((oneOffset < 64) && (bitMarks.test(0) == 0)) {
        bitMarks >>= 1;
        oneOffset += 4; // each bit is for 4 bytes
    }
    // offset to 1st entry
    uint8_t begOffset = oneOffset >> m_sizes[rangeId];

    uint8_t zeroOffset = oneOffset;
    while ((zeroOffset < 64) && (bitMarks.test(0) == 1)) {
        bitMarks >>= 1;
        zeroOffset += 4; // each bit is for 4 bytes
    }

    // 000110:  zeroOffset is 12, oneOffset is 4

    assert(((zeroOffset - oneOffset) % ((uint32_t)1 << m_sizes[rangeId])) == 0);

    uint8_t endOffset = zeroOffset  >> m_sizes[rangeId];

//    std::cout << "  PM:PM begO " << static_cast<int>(begOffset) << " endO " << static_cast<int>(endOffset) << "\n";

    for( int8_t i = begOffset; i < endOffset; ++i) {
        v.insert(i);
    }
}

// TODO: is it beneficial to check trigger range in harden part?
bool descriptiveDataPf::inTriggerRange(uint64_t addr)
{
    return inRange(addr, m_triggerRange);
}

int descriptiveDataPf::getRangeId(uint64_t addr)
{

    for (int i = 0; i < m_ranges.size(); ++i) {
        if (inRange(addr, i))
            return i;
    }
    return -1;
}
bool descriptiveDataPf::inRange(uint64_t addr, uint8_t rangeId)
{
    return ((m_ranges[rangeId].first <= addr) && (addr < m_ranges[rangeId].second));
}

std::bitset<16> descriptiveDataPf::getBitMapOfAddr(uint8_t rangeId, uint64_t addr)
{
    uint64_t offset  = getOffset(addr);
    std::bitset<16> b = m_bitmap[rangeId] << (offset >> 2); // each bit is for 4 bytes
//    std::cout << "PM:PM getBitMapOfAddr o " << std::hex << offset << " " << b << " r " << static_cast<int>(rangeId) << " " << m_bitmap[rangeId] << "\n";
    return b;
}

uint64_t descriptiveDataPf::getAddrAtOffset(uint8_t rangeId, uint64_t addr, uint64_t offset)
{
    uint64_t newAddr = addr + (offset << m_sizes[rangeId]);

    if (!inRange(newAddr, rangeId)) {
        m_numOutOfRanges++;
    }

    // std::cout << "PM:PM getAddrAtOffset r " << std:: hex << static_cast<int>(rangeId)  << " s " << static_cast<int>(m_sizes[rangeId]) << " addr " << addr << " naddr " << newAddr << " o " << offset << std::dec << "\n";
    return newAddr;
}

descriptiveDataPf::descriptiveDataPf(std::istream& afile, std::istream& dfile)
{
    initialize(afile, dfile);
}

descriptiveDataPf::descriptiveDataPf(std::string fRange, std::string fData)
{

    std::ifstream afile; afile.open(fRange);
    if(!afile.is_open()) {
      std::cout << "Unable to open the file " << fRange << std::endl;
      exit(1);
    }

    std::ifstream dfile; dfile.open(fData);
    if(!dfile.is_open()) {
      std::cout << "Unable to open the file " << fData << std::endl;
      exit(1);
    }

    initialize(afile, dfile);

    afile.close();
    dfile.close();
}

descriptiveDataPf::descriptiveDataPf(
    const std::vector<std::tuple<uint64_t, uint64_t, bool, bool, uint64_t, uint64_t, uint64_t>>& info
)
{
    const uint64_t numArrays = info.size();
    m_ranges.resize(numArrays);
    m_sizes.resize(numArrays);
    m_indexTypes.resize(numArrays);
    m_rangeTypes.resize(numArrays);
    m_dataI.resize(numArrays);
    m_dataP.resize(numArrays);
    for (const auto& p: info)
    {
        uint64_t arrayID = std::get<0>(p);
        uint64_t isIndexedByArrayID = std::get<1>(p);
        bool isIndexedByIndex = std::get<2>(p);
        bool isRangedAccess = std::get<3>(p);
        uint64_t startVAddr = std::get<4>(p);
        uint64_t endVAddr = std::get<5>(p);
        uint64_t elementSize = std::get<6>(p);
        uint64_t numElements = (endVAddr - startVAddr) / elementSize;
        m_ranges[arrayID] = std::make_pair(startVAddr, endVAddr);
        m_sizes[arrayID] = (uint64_t)log2(elementSize); // log2 of elementSize
        m_indexTypes[arrayID] = isIndexedByIndex;
        m_rangeTypes[arrayID] = isRangedAccess ? rangeType::range : rangeType::single;
        m_edges[arrayID] = (isIndexedByArrayID == -1ULL) ? -1 : isIndexedByArrayID;
        if (isIndexedByIndex)
        {
            m_dataI[arrayID] = std::vector<uint32_t>(numElements);
        }
        else
        {
            m_dataP[arrayID] = std::vector<uint64_t>(numElements);
        }
        std::cout << "DescriptiveDataPF" \
                  << " arrayID: " << arrayID \
                  << " indexingID: " << (uint64_t)m_edges[arrayID] \
                  << " pointer(0)/index(1): " << m_indexTypes[arrayID] \
                  << " single(0)/range(1): " << isRangedAccess \
                  << " element_size: " << (uint64_t)m_sizes[arrayID] \
                  << " vaddr: [0x" <<std::hex<< m_ranges[arrayID].first \
                  << ", 0x" << m_ranges[arrayID].second << "]"<< std::dec << std::endl;

    }
    m_bitmap.clear(); m_bitmap.resize(m_sizes.size());
    for (int i = 0; i < m_sizes.size(); ++i) {
        std::bitset<16> b;
        switch (m_sizes[i]) {
           case 2: // for 4
                b = 0b1;
                break;
           case 3: // for 8
                b = 0b11;
                break;
           case 4: // for 16
                b = 0b1111;
                break;
        }

        m_bitmap[i] = b;
    }

    m_distance = 1;
}

void descriptiveDataPf::initialize(std::istream& aIstrm, std::istream& dIstrm)
{
    m_distance = 1;

    readAddrRanges(aIstrm);
    readData(dIstrm);
    processData();

    m_bitmap.clear(); m_bitmap.resize(m_sizes.size());
    for (int i = 0; i < m_sizes.size(); ++i) {
        std::bitset<16> b;
        switch (m_sizes[i]) {
           case 2: // for 4
                b = 0b1;
                break;
           case 3: // for 8
                b = 0b11;
                break;
           case 4: // for 16
                b = 0b1111;
                break;
        }

        m_bitmap[i] = b;
    }

//    // for checking
//    std::cout << "PM:PM m_bitmap\n" << std::hex;
//    for (const auto& b : m_bitmap)
//        std::cout << " " << b;
//    std::cout << std::dec << "\n";
}

// Some ranges contain pointer instead of index. The input files always contain indices. This function converts those ranges.
void descriptiveDataPf::processData()
{
    m_dataP.clear(); m_dataP.resize(m_dataI.size());
    for (int i = 0; i < m_dataI.size(); ++i) {
        if (!m_indexTypes[i]) {
            // TODO: remove linearlity assumption
            uint64_t baseAddr = m_ranges[i+1].first; // entry in range i is used to point to entry in range i+1
            for (const auto& d : m_dataI[i])
                m_dataP[i].push_back(baseAddr + (d << m_sizes[i+1]));

            m_dataI[i].clear();
        }
    }

//    // for checking
//    std::cout << "PM:PM processData dataI \n";
//    for (int i = 0; i < m_dataI.size(); ++i) {
//        std::cout << i << " " << m_dataI[i].size() << "\n";
//        for (const auto& dat : m_dataI[i]) {
//            std::cout << " " << dat;
//        }
//        std::cout << "\n";
//    }
//    std::cout << "PM:PM processData dataP \n";
//    for (int i = 0; i < m_dataP.size(); ++i) {
//        std::cout << i << " " << m_dataP[i].size() << "\n" << std::hex;
//        for (const auto& dat : m_dataP[i]) {
//            std::cout << " " << dat;
//        }
//        std::cout << "\n" << std::dec;
//    }
}


// Read address ranges
void descriptiveDataPf::readAddrRanges(std::istream& istrm) {

    struct range_data {
      int8_t   toIdx;
      bool     iType;
      rangeType rType;
      uint64_t begin;
      uint64_t end;
      uint32_t num;
      uint32_t size;

      range_data(int8_t tIdx, bool iT, rangeType rT, uint64_t b,uint64_t e,uint32_t n,uint32_t z)
      : toIdx(tIdx), iType(iT), rType(rT), begin(b), end(e), num(n), size(z)
      {}
    };

    uint8_t l_numRanges = 0;
    std::unordered_map<uint8_t,range_data> l_ranges;



    std::string line;
    int l_lineNum = 1;
    std::regex rgx("\\s+");
    std::sregex_token_iterator end;

    while(getline(istrm, line)) {
      std::vector<std::string> words(std::sregex_token_iterator(line.begin(),line.end(),rgx,-1), end);
      if (words.size() != 9) {
        std::cout << "ERROR: descriptiveDataPf::readAddrRanges find wrong data format at line " << l_lineNum << "\n";
        exit(1);
      }

      uint8_t  idx        = std::stoi(words[1]);
      int8_t   toIdx      = std::stoi(words[2]);
      bool     iType      = (words[3][0] == 'i') ? true : false;
      rangeType rType     = (words[4][0] == 's') ? rangeType::single : rangeType::range;
      uint64_t range_beg  = std::stoul(words[5], nullptr, 16);
      uint64_t range_end  = std::stoul(words[6], nullptr, 16);
      uint64_t num        = std::stoi(words[7]);
      uint64_t entry_size = std::stoi(words[8]);
      l_numRanges = std::max<uint8_t>(l_numRanges, idx+1);
      l_ranges.insert({idx, range_data(toIdx, iType, rType, range_beg, range_end, num, entry_size)});
      l_lineNum++;
    }

    m_ranges.clear(); m_ranges.resize(l_numRanges);
    m_sizes.clear();  m_sizes.resize(l_numRanges);
    m_indexTypes.clear(); m_indexTypes.resize(l_numRanges);
    m_rangeTypes.clear(); m_rangeTypes.resize(l_numRanges);
    for (const auto& entry : l_ranges) {
        m_ranges[entry.first] = std::make_pair(entry.second.begin, entry.second.end);
        // size must be power of 2
        m_sizes[entry.first] = std::log2(entry.second.size); // store as log to allow shifting
        m_indexTypes[entry.first] = entry.second.iType;
        m_rangeTypes[entry.first] = entry.second.rType;
        m_edges[entry.first]      = entry.second.toIdx;
    }

//    // for checking
//    for (int i = 0; i < m_ranges.size(); ++i) {
//        std::cout << "PM:PM descriptiveDataPf::readAddrRanges " << i << " " << (m_indexTypes[i] ? "i" : "p") << " 0x" << std::hex << m_ranges[i].first << " 0x" << m_ranges[i].second << std::dec << " " << static_cast<int>(m_sizes[i]) << std::endl;
//    }
}


  // read reads upto two arrays of edge arrays.
  // An edge array list neighbors of a node as a subarray. The neighbot of node 0 is listed first in the array.
  // There can be upto two arrays: one for outgoing neighbors and the other for incomming ones.
  // Each array start with its type, beginning virtual address of the array, one pass end address and the number of entries.
  // The entries of the associated array is listed one per line. An example of the file is shown below.
  //
  // oneigh 0x2b86bfef 0x2b86d042 68475391
  // 1
  // 3
  // ineigh 0x3b86bfef 0x3b86d042 68475391
  // 7
  // 9
void descriptiveDataPf::readData(std::istream& istrm) {

    uint8_t l_numData = 0;
    std::unordered_map<uint8_t,std::vector<uint32_t>> l_data;

    std::string line;
    int l_lineNum = 1;
    //int line_inc = 0;
    std::regex rgx("\\s+");
    std::sregex_token_iterator end;

    while(getline(istrm, line)) {
//      std::cout<<line<<endl;
      if (isalpha(line[0])) {
        std::vector<std::string> words(std::sregex_token_iterator(line.begin(),line.end(),rgx,-1), end);
        if (words.size() != 3) {
          std::cout << "ERROR: descriptiveDataPf::readData find wrong data format at line " << l_lineNum << "\n";
          exit(1);
        }

        uint8_t  idx = std::stoi(words[1]);
        uint64_t num = std::stoi(words[2]);
        l_numData    = std::max<uint8_t>(l_numData, idx+1);

        std::vector<uint32_t> this_data = readDataArray(istrm, num);
        l_data.insert({idx, this_data});
        l_lineNum += num+1;
      } else {
          std::cout << "ERROR: descriptiveDataPf::readData find wrong data format at line " << l_lineNum << "\n";
      }
    }

    m_dataI.clear(); m_dataI.resize(l_numData);
    for (const auto& entry : l_data) {
        m_dataI[entry.first] = entry.second;
    }

//    // for checking
//    std::cout << "PM:PM descriptiveDataPf::readData\n";
//    for (int i = 0; i < m_dataI.size(); ++i) {
//        std::cout << i << " " << m_dataI[i].size() << "\n";
//        for (const auto& dat : m_dataI[i]) {
//            std::cout << " " << dat;
//        }
//        std::cout << "\n";
//    }
}


  // read_data reads lines from the given istram handle for the given number of line. Only one integer is expected per line.
std::vector<uint32_t> descriptiveDataPf::readDataArray(std::istream &istrm, int num) {
    std::vector<uint32_t> data(num,0);
    std::string line;
    int i = 0;
    while((i < num) && getline(istrm, line)) {
      // Within this section of the file, a line is expected to have only one value.
      data[i++] = std::stoi(line);
    }
    return data;
}

void descriptiveDataPf::printAddrRange(std::ostream& ostrm)
{
    for (int i = 0; i < m_ranges.size(); ++i) {
        ostrm << i << (m_indexTypes[i] ? " i 0x" : " p 0x") << std::hex << m_ranges[i].first << " 0x" << m_ranges[i].second << std::dec << " " << static_cast<int>(m_sizes[i]) << "\n";
    }
}

int descriptiveDataPf::testBfs()
{
    int err = 0;

    err += testSimpleGraph();
    err += testNonConsecutiveIndexPattern();
    err += testConsecutiveRange();
    err += testRange2CL();
    std::cout << "------------ PM:PM descriptiveDataPf::testBfs the number of errors found is " << err << "\n";
    return err;
}

// Test only something specific to Pr. Testing general features is done in testBfs().
int descriptiveDataPf::testPr()
{
    int err = 0;

    std::cout << "\nPM:PM testPr\n";
    std::cout << "\nPM:PM workqueue req\n";
    // first workqueue entry
    uint64_t wqe1 = 0xa754a0;
    requestEntry wq1(0, wqe1, n_PageSize::e_Page4K, 0, 0); // type and gid always 0 in the test
    wq1.write(std::cout, "PM:PM req node 1");
    std::vector<responseEntry> grp = operateReq(wq1);
    if (grp[0] != responseEntry(0, 0xa75480, 0, 0xa754a8, 0b0000110000000000, n_PageSize::e_Page4K, 0)) {
        grp[0].write(std::cout,"Got entry");
        std::cout << "***ERROR! req: response entry is wrong" << "\n";
        ++err;
    }

    std::cout << "\nPM:PM spaning two cacheline req\n";
    // first workqueue entry
    uint64_t wqe2 = 0xa754b0;
    requestEntry wq2(0, wqe2, n_PageSize::e_Page4K, 0, 0); // type and gid always 0 in the test
    wq2.write(std::cout, "PM:PM req node 1");
    grp = operateReq(wq2);
    for (const auto& e : grp)
        e.write(std::cout,"");
    std::sort( grp.begin( ), grp.end( ), [ ]( const responseEntry& lhs, const responseEntry& rhs ) {return lhs.bvaddr < rhs.bvaddr;});
    if (grp[0] != responseEntry(0, 0xa75480, 0, 0xa754b8, 0b1100000000000000, n_PageSize::e_Page4K, 0, false, 0)) {
        grp[0].write(std::cout,"Got entry");
        std::cout << "***ERROR! req: response entry is wrong" << "\n";
        ++err;
    }
    if (grp[1] != responseEntry(0, 0xa754c0, 0, 0xa754b8, 0b0000000000000000, n_PageSize::e_Page4K, 0, true, 4)) {
        grp[1].write(std::cout,"Got entry");
        std::cout << "***ERROR! req: response entry is wrong" << "\n";
        ++err;
    }

    std::cout << "------------ PM:PM descriptiveDataPf::testPr the number of errors found is " << err << "\n";
    return err;
}

std::istringstream descriptiveDataPf::descriptiveDataPf::addrRangeExPr()
{
    std::string addr =
    "iindex 0 1 p r 0xa75498 0xa754d8 8 8\n" // for after rev 158891, before that use oindex 1 p 0xa75490 0xa754d0 8 8
    "ineigh 1 2 i s 0xa75450 0xa75488 14 4\n"
    "nprop 2 -1 i s 0xa74970 0xa7498c 7 4\n";
    std::istringstream iss;
    iss.str(addr);
    return iss;
}

std::istringstream descriptiveDataPf::descriptiveDataPf::dataExPr()
{
    std::string data =
    "iindex 0 8\n"
    "0\n"
    "1\n"
    "3\n"
    "6\n"
    "7\n"
    "10\n"
    "11\n"
    "14\n"
    "ineigh 1 14\n"
    "1\n"
    "2\n"
    "6\n"
    "0\n"
    "3\n"
    "5\n"
    "1\n"
    "0\n"
    "2\n"
    "3\n"
    "4\n"
    "1\n"
    "2\n"
    "5\n";
    std::istringstream iss;
    iss.str(data);
    return iss;
}

std::istringstream descriptiveDataPf::descriptiveDataPf::addrRangeExBfs()
{
    std::string addr =
    "wqueue 0 1 i s 0xa75570 0xa7558c 7 4\n"
    "oindex 1 2 p r 0xa75498 0xa754d8 8 8\n" // for after rev 158891, before that use oindex 1 p 0xa75490 0xa754d0 8 8
    "oneigh 2 3 i s 0xa75450 0xa75488 14 4\n"
    "nprop 3 -1 i s 0xa74970 0xa7498c 7 4\n";
    std::istringstream iss;
    iss.str(addr);
    return iss;
}

std::istringstream descriptiveDataPf::descriptiveDataPf::dataExBfs()
{
    std::string data =
    "wqueue 0 7\n"
    "2\n"
    "1\n"
    "4\n"
    "6\n"
    "0\n"
    "5\n"
    "3\n"
    "oindex 1 8\n"
    "0\n"
    "2\n"
    "5\n"
    "8\n"
    "10\n"
    "11\n"
    "13\n"
    "14\n"
    "oneigh 2 14\n"
    "2\n"
    "4\n"
    "0\n"
    "3\n"
    "6\n"
    "1\n"
    "4\n"
    "6\n"
    "2\n"
    "4\n"
    "5\n"
    "2\n"
    "6\n"
    "1\n";
    std::istringstream iss;
    iss.str(data);
    return iss;
}

int descriptiveDataPf::testRange2CL()
{
    int err = 0;
    std::cout << "\n\nPM:PM testRange2CL\n";
    std::cout << "PM:PM process range beg\n";
    std::vector<responseEntry> grp = operateRsp(responseEntry(0, 0xa75480, 1,  0xa75574, 0b1100000000000000, n_PageSize::e_Page4K, 0));
    if (grp[0] != responseEntry(0, 0xa754c0, 1,  0xa75574 , 0b0000000000000000, n_PageSize::e_Page4K, 0, true, 4)) {
        grp[0].write(std::cout,"Got entry");
        std::cout << "***ERROR! res : response entry is wrong" << "\n";
        ++err;
    }

    std::cout << "\nPM:PM process range spill\n";
    grp = operateRsp(grp[0]);
    if (grp[0] != responseEntry(0, 0xa75440, 2,  0xa75574 , 0b0100000000000000, n_PageSize::e_Page4K, 0)) {
        grp[0].write(std::cout,"Got entry");
        std::cout << "***ERROR! res : response entry is wrong" << "\n";
        ++err;
    }

    std::cout << "\nPM:PM process range at lsb\n";
    grp = operateRsp(responseEntry(0, 0xa754c0, 1,  0xa75574 , 0b0000000000000011, n_PageSize::e_Page4K, 0, false, 4));
    std::sort( grp.begin( ), grp.end( ), [ ]( const responseEntry& lhs, const responseEntry& rhs ) {return lhs.bvaddr < rhs.bvaddr;});
    if (grp[0] != responseEntry(0, 0xa75440, 2,  0xa75574 , 0b1000000000000000, n_PageSize::e_Page4K, 0)) {
        grp[0].write(std::cout,"Got entry");
        std::cout << "***ERROR! res : response entry is wrong" << "\n";
        ++err;
    }
    if (grp[1] != responseEntry(0, 0xa75480, 2,  0xa75574 , 0b0000000000000001, n_PageSize::e_Page4K, 0)) {
        grp[1].write(std::cout,"Got entry");
        std::cout << "***ERROR! res : response entry is wrong" << "\n";
        ++err;
    }

    std::cout << "\nPM:PM process range spill + continuation\n";
    grp = operateRsp(responseEntry(0, 0xa754c0, 1,  0xa75574 , 0b0000000000000011, n_PageSize::e_Page4K, 0, true, 4));
    std::sort( grp.begin( ), grp.end( ), [ ]( const responseEntry& lhs, const responseEntry& rhs ) {return lhs.bvaddr < rhs.bvaddr;});
    if (grp[0] != responseEntry(0, 0xa75440, 2,  0xa75574 , 0b1100000000000000, n_PageSize::e_Page4K, 0)) {
        grp[0].write(std::cout,"Got entry");
        std::cout << "***ERROR! res : response entry is wrong" << "\n";
        ++err;
    }
    if (grp[1] != responseEntry(0, 0xa75480, 2,  0xa75574 , 0b0000000000000001, n_PageSize::e_Page4K, 0)) {
        grp[1].write(std::cout,"Got entry");
        std::cout << "***ERROR! res : response entry is wrong" << "\n";
        ++err;
    }

//
    std::cout << "\n\nPM:PM process range spill + other\n";
    grp = operateRsp(responseEntry(0, 0xa754c0, 1,  0xa75574 , 0b0000000000001100, n_PageSize::e_Page4K, 0, true, 4));
    std::sort( grp.begin( ), grp.end( ), [ ]( const responseEntry& lhs, const responseEntry& rhs ) {return lhs.bvaddr < rhs.bvaddr;});
    if (grp[0] != responseEntry(0, 0xa75440, 2,  0xa75574 , 0b0100000000000000, n_PageSize::e_Page4K, 0)) {
        grp[0].write(std::cout,"Got entry");
        std::cout << "***ERROR! res : response entry is wrong" << "\n";
        ++err;
    }
    if (grp[1] != responseEntry(0, 0xa75480, 2,  0xa75574 , 0b0000000000000010, n_PageSize::e_Page4K, 0)) {
        grp[1].write(std::cout,"Got entry");
        std::cout << "***ERROR! res : response entry is wrong" << "\n";
        ++err;
    }

    std::cout << "\nPM:PM process range at lsb + other\n";
    grp = operateRsp(responseEntry(0, 0xa754c0, 1,  0xa75574 , 0b0000000000001111, n_PageSize::e_Page4K, 0, false, 4));
    std::sort( grp.begin( ), grp.end( ), [ ]( const responseEntry& lhs, const responseEntry& rhs ) {return lhs.bvaddr < rhs.bvaddr;});
    if (grp[0] != responseEntry(0, 0xa75440, 2,  0xa75574 , 0b1000000000000000, n_PageSize::e_Page4K, 0)) {
        grp[0].write(std::cout,"Got entry");
        std::cout << "***ERROR! res : response entry is wrong" << "\n";
        ++err;
    }
    if (grp[1] != responseEntry(0, 0xa75480, 2,  0xa75574 , 0b0000000000000011, n_PageSize::e_Page4K, 0)) {
        grp[1].write(std::cout,"Got entry");
        std::cout << "***ERROR! res : response entry is wrong" << "\n";
        ++err;
    }

    std::cout << "\nPM:PM process range spill + continuation + other\n";
    grp = operateRsp(responseEntry(0, 0xa754c0, 1,  0xa75574 , 0b0000000000001111, n_PageSize::e_Page4K, 0, true, 4));
    std::sort( grp.begin( ), grp.end( ), [ ]( const responseEntry& lhs, const responseEntry& rhs ) {return lhs.bvaddr < rhs.bvaddr;});
    if (grp[0] != responseEntry(0, 0xa75440, 2,  0xa75574 , 0b1100000000000000, n_PageSize::e_Page4K, 0)) {
        grp[0].write(std::cout,"Got entry");
        std::cout << "***ERROR! res : response entry is wrong" << "\n";
        ++err;
    }
    if (grp[1] != responseEntry(0, 0xa75480, 2,  0xa75574 , 0b0000000000000011, n_PageSize::e_Page4K, 0)) {
        grp[1].write(std::cout,"Got entry");
        std::cout << "***ERROR! res : response entry is wrong" << "\n";
        ++err;
    }
    return err;
}

int descriptiveDataPf::testConsecutiveRange()
{
    int err = 0;
    std::cout << "\nPM:PM testConsecutiveRange\n";
    std::vector<responseEntry> grp = operateRsp(responseEntry(0, 0xa75480, 1,  0xa75574, 0b0011001111000000, n_PageSize::e_Page4K, 0));
    if (grp[0] != responseEntry(0, 0xa75440, 2,  0xa75574,   0b0011000111110000, n_PageSize::e_Page4K, 0)) {
        grp[0].write(std::cout,"Got entry");
        std::cout << "***ERROR! req: response entry is wrong" << "\n";
        ++err;
    }
    return err;
}

int descriptiveDataPf::testNonConsecutiveIndexPattern()
{
    int err = 0;
    std::cout << "\nPM:PM testNonConsecutivePattern\n";
    std::vector<responseEntry> grp = operateRsp(responseEntry(0, 0xa75440, 2,  0xa75554, 0b0001011001000000, n_PageSize::e_Page4K, 0));
    std::sort( grp.begin( ), grp.end( ), [ ]( const responseEntry& lhs, const responseEntry& rhs ) {return lhs.bvaddr < rhs.bvaddr;});
    if (grp[0] != responseEntry(0, 0xa74940, 3,  0xa75554 , 0b0111000000000000, n_PageSize::e_Page4K, 0)) {
        grp[0].write(std::cout,"Got entry");
        std::cout << "***ERROR! res : response entry is wrong" << "\n";
        ++err;
    }
    if (grp[1] != responseEntry(0, 0xa74980, 3,  0xa75554 , 0b0000000000000001, n_PageSize::e_Page4K, 0)) {
        grp[1].write(std::cout,"Got entry");
        std::cout << "***ERROR! res : response entry is wrong" << "\n";
        ++err;
    }
    return err;
}

int descriptiveDataPf::testSimpleGraph()
{
    int err = 0;
    std::cout << "\nPM:PM testSimpleGraph\n";
    std::cout << "\nPM:PM workqueue req\n";
    // first workqueue entry
    uint64_t wqe1 = 0xa75570;
    requestEntry wq1(0, wqe1, n_PageSize::e_Page4K, 0, 0); // type and gid always 0 in the test
    wq1.write(std::cout, "PM:PM req node 1");
    std::vector<responseEntry> grp = operateReq(wq1);
    if (grp[0] != responseEntry(0, 0xa75540, 0,  0xa75574, 0b0010000000000000, n_PageSize::e_Page4K, 0)) {
        grp[0].write(std::cout,"Got entry");
        std::cout << "***ERROR! req: response entry is wrong" << "\n";
        ++err;
    }

    std::cout << "\nPM:PM workqueue rsp\n";
    // rcv read for workQueue
    grp = operateRsp(grp[0]);
    if (grp[0] != responseEntry(0, 0xa75480, 1,  0xa75574, 0b0000001100000000, n_PageSize::e_Page4K, 0)) {
        grp[0].write(std::cout,"Got entry");
        std::cout << "***ERROR! res wq : response entry is wrong" << "\n";
        ++err;
    }

    std::cout << "\nPM:PM offsetList rsp\n";
    // rcv read for offsetList
    grp = operateRsp(grp[0]);
    if (grp[0] != responseEntry(0, 0xa75440, 2,  0xa75574,0b0000000111000000 , n_PageSize::e_Page4K, 0)) {
        grp[0].write(std::cout,"Got entry");
        std::cout << "***ERROR! res ol : response entry 3 is wrong" << "\n";
        ++err;
    }

    std::cout << "\nPM:PM edgeList rsp\n";
    // rcv read for offsetList
    grp = operateRsp(grp[0]);
    if (grp.size()!=2)     {
        std::cout << "***ERROR! res el : expect response entry to have 2 entries" << "\n";
        ++err;
    }
    std::sort( grp.begin( ), grp.end( ), [ ]( const responseEntry& lhs, const responseEntry& rhs ) {return lhs.bvaddr < rhs.bvaddr;});
    if (grp[0] != responseEntry(0, 0xa74940, 3,  0xa75574, 0b1001000000000000, n_PageSize::e_Page4K, 0)) {
        grp[0].write(std::cout,"Got entry");
        std::cout << "***ERROR! res el : response entry is wrong" << "\n";
        ++err;
    }
    if (grp[1] != responseEntry(0, 0xa74980, 3,  0xa75574, 0b0000000000000100 , n_PageSize::e_Page4K, 0)) {
        grp[1].write(std::cout,"Got entry");
        std::cout << "***ERROR! res el : response entry is wrong" << "\n";
        ++err;
    }

    std::cout << "\nPM:PM visitList rsp\n";
    // rcv read for offsetList
    std::vector<responseEntry> grp2 = operateRsp(grp[0]);
    if (grp2.size()!=0)    {
        std::cout << "***ERROR! expect response entry to have 0 entries" << "\n";
        ++err;
    }
    grp2 = operateRsp(grp[1]);
    if (grp2.size()!=0)    {
        std::cout << "***ERROR! expect response entry to have 0 entries" << "\n";
        ++err;
    }
    return err;
}
