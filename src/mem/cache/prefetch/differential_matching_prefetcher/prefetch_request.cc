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

#include "mem/cache/prefetch/differential_matching_prefetcher/prefetch_request.hh"

namespace gem5
{

namespace prefetch
{

namespace dmp
{

PrefetchRequest::PrefetchRequest(
    const Addr _target_pc, const Addr _prefetch_vaddr,
    const uint64_t _size, const uint64_t _irt_id
) : response(0xBADC0DE),
    queue_entering_tick(0),
    start_address_translation_tick(0),
    end_address_translation_tick(0),
    memory_request_issued_tick(0),
    memory_request_complete_tick(0),
    target_pc(_target_pc),
    prefetch_vaddr(_prefetch_vaddr),
    size(_size),
    irt_id(_irt_id)
{
}

void
PrefetchRequest::setResponse(const uint64_t _response)
{
    response = _response;

}

bool
PrefetchRequest::setResponseFromCacheBlockData(
    const uint8_t* cache_block_data, const uint64_t cache_block_size
)
{
    // Calculate the offset of the requested data within the cache block
    const uint64_t offset_to_cache_block =
        prefetch_vaddr & (cache_block_size - 1);
    assert(offset_to_cache_block + size <= cache_block_size);

    const uint8_t* data_ptr = cache_block_data + offset_to_cache_block;
    if (size == 1) {
        uint8_t data = *data_ptr;
        setResponse(static_cast<uint64_t>(data));
    } else if (size == 2) {
        uint16_t data = *reinterpret_cast<const uint16_t*>(data_ptr);
        setResponse(static_cast<uint64_t>(data));
    } else if (size == 4) {
        uint32_t data = *reinterpret_cast<const uint32_t*>(data_ptr);
        setResponse(static_cast<uint64_t>(data));
    } else if (size == 8) {
        uint64_t data = *reinterpret_cast<const uint64_t*>(data_ptr);
        setResponse(data);
    } else {
        // Unsupported data size
        return false;
    }
    return true;
}

uint64_t
PrefetchRequest::getResponse() const
{
    return response;
}

} // namespace dmp

} // namespace prefetch

} // namespace gem5
