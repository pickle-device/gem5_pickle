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

#ifndef __DMP_UTIL_HH__
#define __DMP_UTIL_HH__

#include <cstdint>
#include <queue>
#include <unordered_set>

#include "base/types.hh"

namespace gem5
{

namespace prefetch
{

namespace dmp
{

constexpr uint64_t log2(uint64_t n) {
    return (n < 2) ? 0 : 1 + log2(n / 2);
}

// A FIFO that allows querying for the presence of an element in O(1) time.
// Implemented as a combination of a queue and a hash set. The queue maintains
// the order of the elements, while the hash set allows for O(1) presence
// query.
class QueuedDict
{
    private:
        std::queue<Addr> queue;
        std::unordered_set<Addr> set;
        size_t capacity;
    public:
        QueuedDict(size_t _capacity) : capacity(_capacity) {}
        bool isFull() const {
            return queue.size() >= capacity;
        }
        bool contains(const Addr& addr) const {
            return set.find(addr) != set.end();
        }
        void push(const Addr& addr) {
            if (isFull()) {
                const Addr& front = queue.front();
                set.erase(front);
                queue.pop();
            }
            queue.push(addr);
            set.insert(addr);
        }
};  // class QueuedDict

} // namespace dmp

} // namespace prefetch

} // namespace gem5

#endif // __DMP_UTIL_HH__
