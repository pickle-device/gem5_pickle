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

#include "mem/cache/prefetch/differential_matching_prefetcher/index_queue.hh"

#include <optional>
#include <sstream>

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/DifferentialMatchingPrefetcherIndexQueueDebug.hh"

namespace gem5
{

namespace prefetch
{

IndexQueueEntry::IndexQueueEntry(Addr _pc, Tick _access_timestamp)
  : pc(_pc), access_timestamp(_access_timestamp),
    tried_count(0), matched_count(0)
{
}

void
IndexQueueEntry::profileTried()
{
    tried_count++;
}

void
IndexQueueEntry::profileMatched()
{
    matched_count++;
}

// See Section III.D of the paper for more details
double
IndexQueueEntry::getScore() const
{
    if (tried_count == 0) {
        return 3.0; // the score is guaranteed to be the highest
    }
    return 1.0 * (matched_count + 1) / tried_count;
}

IndexQueue::IndexQueue(
    uint64_t _max_size, IndexQueueReplacementPolicy _replacement_policy
)
  : max_size(_max_size), replacement_policy(_replacement_policy)
{
    index_queue.reserve(max_size);
}

void
IndexQueue::replaceLeastRecentlyUsedEntry(
    const Addr pc, const Tick access_timestamp
)
{
    if (index_queue.empty()) {
        return;
    }

    auto lru_it = index_queue.begin();
    for (auto it = index_queue.begin(); it != index_queue.end(); ++it) {
        if (it->access_timestamp < lru_it->access_timestamp) {
            lru_it = it;
        }
    }

    // replace the LRU entry
    *lru_it = std::move(IndexQueueEntry(pc, access_timestamp));
}

void
IndexQueue::replaceLowestScoreEntry(
    const Addr pc, const Tick access_timestamp
)
{
    if (index_queue.empty()) {
        return;
    }

    auto lowest_score_it = index_queue.begin();
    for (auto it = index_queue.begin(); it != index_queue.end(); ++it) {
        if (it->getScore() < lowest_score_it->getScore()) {
            lowest_score_it = it;
        }
    }

    // replace the lowest score entry
    *lowest_score_it = std::move(IndexQueueEntry(pc, access_timestamp));
}

void
IndexQueue::add(const Addr pc, const Tick access_timestamp)
{
    bool isUpdated = false;

    // First, check if the PC already exists in the index queue
    for (auto it = index_queue.begin(); it != index_queue.end(); ++it) {
        if (it->pc == pc) {
            // Update existing entry's timestamp
            it->access_timestamp = access_timestamp;
            isUpdated = true;
            break;
        }
    }
    if (isUpdated) {
        return;
    }

    // If the PC was not found and if there is still space, add the new entry
    if (index_queue.size() < max_size) {
        index_queue.emplace_back(pc, access_timestamp);
        return;
    }

    // If the queue is full, replace an entry based on the replacement policy
    if (replacement_policy == IndexQueueReplacementPolicy::LowestScore) {
        replaceLowestScoreEntry(pc, access_timestamp);
    } else if (replacement_policy == IndexQueueReplacementPolicy::LRU) {
        replaceLeastRecentlyUsedEntry(pc, access_timestamp);
    } else {
        panic("Unknown IndexQueue replacement policy!");
    }

    DMP_INDEX_QUEUE_DEBUG(
        "Added PC: %#x, Timestamp: %d\n", pc, access_timestamp
    );
}

bool
IndexQueue::isFull() const
{
    return index_queue.size() == max_size;
}

std::optional<std::vector<Addr>>
IndexQueue::getHighestScorePcs() const
{
    // If the index queue is empty, return nullopt
    if (index_queue.empty()) {
        return std::nullopt;
    }

    // Find the highest score
    double highest_score = index_queue.begin()->getScore();
    for (auto it = index_queue.begin(); it != index_queue.end(); ++it) {
        if (it->getScore() > highest_score) {
            highest_score = it->getScore();
        }
    }

    // Find all entries with the highest score
    std::vector<Addr> highest_score_pcs;
    for (const auto &entry : index_queue) {
        if (entry.getScore() == highest_score) {
            highest_score_pcs.push_back(entry.pc);
        }
    }

    std::stringstream strm;
    strm << "Highest score: " << highest_score << "; PCs: ";

    for (const auto &pc : highest_score_pcs) {
        strm << " 0x" << std::hex << pc << std::dec;
    }
    strm << "\n";
    DMP_INDEX_QUEUE_DEBUG("%s", strm.str().c_str());

    return highest_score_pcs;
}

void
IndexQueue::profileTriedCount(const Addr index_pc)
{
    for (auto &entry : index_queue) {
        if (entry.pc == index_pc) {
            entry.profileTried();
            return;
        }
    }
}

void
IndexQueue::profileMatchedPc(const Addr index_pc)
{
    for (auto &entry : index_queue) {
        if (entry.pc == index_pc) {
            entry.profileMatched();
            return;
        }
    }
}

} // namespace prefetch

} // namespace gem5
