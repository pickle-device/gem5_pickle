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

#ifndef INCLUDED_RESPONSEENTRY_HPP
#define INCLUDED_RESPONSEENTRY_HPP

#include <bitset>
#include <cassert>
#include <cstdint>
#include <functional>


namespace n_PageSize
{
    class t_PageSize
    {
        private:
            uint64_t val = 0;
        public:
            t_PageSize() {}
            t_PageSize(const uint64_t& val) { this->val = val; }
            t_PageSize(const t_PageSize& other) { this->val = other.val; }
            bool operator==(const t_PageSize& other) const {
                return this->val == other.val;
            }
    };
    static const t_PageSize e_Page4K = t_PageSize(1 << 12);
}; // namespace n_PageSize

typedef uint8_t t_Gtid;

struct responseEntry
{
    uint8_t             type; // TODO: can two type produce prefetch in the same cacheline? probably true.
    uint64_t            bvaddr;  // *** can be physical address in core clock domain ***
    // Rightnow just do it for descriptiveDataPf, later need to make entry below a class and use union. Note union does not allow class with non-trivial ctor.
    uint8_t             curRangeIdx; // TODO: can two curRangeIdx produce prefetch in the same cacheline? probably true.
    uint64_t            triggerAddr; // TODO: can two trigger address produce prefetch in the same cacheline? Likely true.
    // For range type, only the beginning of a range is marked.
    // Even when the range end spill over to another cache line, meta is used to indicate a continuation from previoius line. If the LSB position is marked, it is the beg of another range!.
    std::bitset<16>     bitMarks; // 4B granularity
    n_PageSize::t_PageSize pageSize;
    uint8_t             gtid;

    // TODO This is a costly way to forward beg of a range when the end is in another cacheline. Do this only in simulation and keep stat to design a more efficient way in hardware.
    bool                metaValid;
    uint32_t            meta; // beg index in destination range

    responseEntry() {};
    responseEntry(uint8_t i_type, uint64_t i_bvaddr, uint8_t i_curRangeIdx, uint64_t i_triggerAddr, std::bitset<16> i_bitMarks, n_PageSize::t_PageSize i_pageSize, t_Gtid i_gtid, bool i_metaValid = 0, uint32_t i_meta = 0)
        : type(i_type), bvaddr(i_bvaddr), curRangeIdx(i_curRangeIdx), triggerAddr(i_triggerAddr), bitMarks(i_bitMarks), pageSize(i_pageSize), gtid(i_gtid), metaValid(i_metaValid), meta(i_meta) {};
    void write(std::ostream& ostrm, const std::string& msg) const {
        ostrm << msg << " rsp t " << static_cast<int>(type) << " r " << static_cast<int>(curRangeIdx) << std::hex << " bva 0x" << bvaddr << " tga 0x" << triggerAddr << " bm " <<  bitMarks;
        if (metaValid)
            ostrm << " meta " << meta;
        ostrm << std::dec << "\n";
    }
    bool operator==(const responseEntry& b) const{
        if (type != b.type) return false;
        else if (bvaddr != b.bvaddr) return false;
        else if (curRangeIdx != b.curRangeIdx) return false;
        else if (triggerAddr != b.triggerAddr) return false;
        else return true;
//        if (type != b.type) return false;
//        else if (bvaddr != b.bvaddr) return false;
//        else if (curRangeIdx != b.curRangeIdx) return false;
//        else if (triggerAddr != b.triggerAddr) return false;
//        else if (bitMarks != b.bitMarks) return false;
//        else if (pageSize != b.pageSize) return false;
//        else if (gtid != b.gtid) return false;
//        else if (metaValid != b.metaValid) return false;
//        else if (meta != b.meta) return false;
//        else return true;
    }
    bool operator!=(const responseEntry& b) const{
        return !(this->operator==(b));
    }
    void merge(responseEntry b) {
        assert(bvaddr == b.bvaddr); // every field must be equal, except bitMarks
        bitMarks |= b.bitMarks;
        if (!metaValid) { // don't overwrite a.meta
            metaValid = b.metaValid;
            meta      = b.meta;
        }
    }
    bool mergeable(responseEntry b) {
        // TODO
        return (bvaddr == b.bvaddr);
    }
};


struct requestEntry
{
    uint8_t                type;
    uint64_t               bvaddr; // This is not block address. the trigger must point to an entry in the trigger array
    n_PageSize::t_PageSize pageSize;
    uint8_t                gtid;
    uint64_t               rip;  // only for debugging

    requestEntry(){};
    requestEntry(uint8_t i_type, uint64_t i_bvaddr, n_PageSize::t_PageSize i_pageSize, t_Gtid i_gtid, uint64_t i_rip)
        : type(i_type), bvaddr(i_bvaddr), pageSize(i_pageSize), gtid(i_gtid), rip(i_rip) {};
    void write(std::ostream& ostrm, const std::string& msg) const {
        ostrm << msg << " req t " << static_cast<int>(type) << std::hex << " bva 0x" << bvaddr << std::dec << "\n";
    }
    bool operator==(const requestEntry& b) const{
        if (type != b.type) return false;
        else if (bvaddr != b.bvaddr) return false;
        else if (!(pageSize == b.pageSize)) return false;
        else if (gtid != b.gtid) return false;
        else return true;
    }
    bool operator!=(const requestEntry& b) const{
        return !(this->operator==(b));
    }

};


#endif // !INCLUDED_RESPONSEENTRY_HPP
