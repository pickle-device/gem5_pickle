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
template <class T>
class QueuedSet
{
    private:
        std::queue<T> queue;
        std::unordered_set<T> set;
        size_t capacity;
    public:
        QueuedSet(size_t _capacity) : capacity(_capacity) {}
        bool isFull() const {
            if (capacity == 0) {
                return false;
            }
            return queue.size() >= capacity;
        }
        bool contains(const T& element) const {
            return set.find(element) != set.end();
        }
        void push(const T& element) {
            if (isFull()) {
                pop();
            }
            queue.push(element);
            set.insert(element);
        }
        T front() const {
            return queue.front();
        }
        void pop() {
            T old_element = queue.front();
            set.erase(old_element);
            queue.pop();
        }
};  // class QueuedSet

template <class Key, class Value>
class QueuedDict
{
    private:
        std::queue<std::pair<Key, Value>> queue;
        std::unordered_set<Key> set;
        size_t capacity;
    public:
        QueuedDict(size_t _capacity) : capacity(_capacity) {}
        bool isFull() const {
            if (capacity == 0) {
                return false;
            }
            return queue.size() >= capacity;
        }
        bool empty() const {
            return queue.empty();
        }
        bool contains(const Key& key) const {
            return set.find(key) != set.end();
        }
        void push(const std::pair<Key, Value>& pair) {
            if (isFull()) {
                pop();
            }
            queue.push(pair);
            set.insert(pair.first);
        }
        std::pair<Key, Value>& front() {
            return queue.front();
        }
        void pop() {
            auto [old_key, old_value] = queue.front();
            set.erase(old_key);
            queue.pop();
        }
};  // class QueuedDict

} // namespace dmp

} // namespace prefetch

} // namespace gem5

#endif // __DMP_UTIL_HH__
