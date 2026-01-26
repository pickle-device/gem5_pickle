/*
 * Copyright (c) 2026 The Regents of the University of California
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __DMP_INDEX_QUEUE_HH__
#define __DMP_INDEX_QUEUE_HH__

#include <cstdint>
#include <optional>
#include <vector>

#include "base/types.hh"

namespace gem5
{

namespace prefetch
{

class IndexQueueEntry
{
  public:
    Addr pc;
    Tick access_timestamp;
    uint64_t tried_count;
    uint64_t matched_count;

    IndexQueueEntry(Addr _pc, Tick _access_timestamp);
    void profileTried();
    void profileMatched();
    double getScore() const;
};

enum class IndexQueueReplacementPolicy
{
    LRU,
    LowestScore
};

class IndexQueue
{
  private:
    const uint64_t max_size;
    const IndexQueueReplacementPolicy replacement_policy;
    std::vector<IndexQueueEntry> index_queue;
    void replaceLeastRecentlyUsedEntry(
        const Addr pc, const Tick access_timestamp
    );
    void replaceLowestScoreEntry(
        const Addr pc, const Tick access_timestamp
    );
  public:
    IndexQueue(
      uint64_t _max_size, IndexQueueReplacementPolicy _replacement_policy);
    void add(const Addr pc, const Tick access_timestamp);
    bool isFull() const;
    // According to the paper's Section 4.3, DMP only picks the PC with the
    // highest score for differential matching. However, since there could be
    // multiple PCs with the same highest score, we need to return a vector of
    // PCs. Why? Among multiple PCs with the same highest score, without
    // a mechanism to break the tie, we repeatedly pick the same PC in every
    // promotion round, starving other PCs from being promoted. If a PC is
    // already in the ICS, promoting it again is useless. Therefore, by
    // returning all PCs with the highest score, we give a chance to the
    // prefetcher to pick a different PC that is not in the ICS yet.
    std::optional<std::vector<Addr>> getHighestScorePcs() const;
    // If the PC is inserted to the ICS, we increase its tried count.
    void increaseTriedCount(const Addr pc);
};

} // namespace prefetch

} // namespace gem5

#endif // __DMP_INDEX_QUEUE_HH__
