/*
 * Copyright (c) 2025 The Regents of the University of California
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
**/

#ifndef PICKLE_JOB_HH
#define PICKLE_JOB_HH

#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace gem5
{

class PickleJobArrayDescriptor
{
    public:
        uint64_t array_id;
        uint64_t dst_id;
        uint64_t vaddr_start;
        uint64_t vaddr_end;
        uint64_t element_size;
        bool is_ranged_access; // single element or ranged access
        bool is_index_access; // pointer or index access

        PickleJobArrayDescriptor(
            uint64_t array_id, uint64_t dst_id, uint64_t vaddr_start,
            uint64_t vaddr_end, uint64_t element_size,
            bool is_ranged_access, bool is_index_access
        ) : array_id(array_id), dst_id(dst_id), vaddr_start(vaddr_start),
            vaddr_end(vaddr_end), element_size(element_size),
            is_ranged_access(is_ranged_access),
            is_index_access(is_index_access) {}
        std::string to_string() const
        {
            std::stringstream ss;
            ss << "array_id: " << array_id << ", dst_id: " << dst_id
               << ", vaddr_start: 0x" << std::hex << vaddr_start << std::dec
               << ", vaddr_end: 0x" << std::hex << vaddr_end << std::dec
               << ", element_size: " << element_size
               << ", is_ranged_access: " << is_ranged_access
               << ", is_index_access: " << is_index_access;
            return ss.str();
        }
}; // class PickleJob

class PickleJobCollection
{
    public:
        std::vector<PickleJobArrayDescriptor> arrays;
        PickleJobCollection() {}
        PickleJobCollection(std::vector<uint8_t> &job_ptr)
        {
            // job_ptr[0] is the number of arrays
            const uint8_t num_arrays = job_ptr[0];
            arrays.reserve(num_arrays);
            uint64_t job_ptr_i = 1;
            for (uint64_t i = 0; i < num_arrays; i++) {
                uint64_t array_id = *(uint64_t *)&job_ptr[job_ptr_i];
                job_ptr_i += 8;
                // dst_indexing_array_id
                uint64_t dst_id = *(uint64_t *)&job_ptr[job_ptr_i];
                job_ptr_i += 8;
                // vaddr_start
                uint64_t vaddr_start = *(uint64_t *)&job_ptr[job_ptr_i];
                job_ptr_i += 8;
                // vaddr_end
                uint64_t vaddr_end = *(uint64_t *)&job_ptr[job_ptr_i];
                job_ptr_i += 8;
                // element_size
                uint64_t element_size = *(uint64_t *)&job_ptr[job_ptr_i];
                job_ptr_i += 8;
                // access_type
                uint64_t is_ranged_access = *(uint64_t *)&job_ptr[job_ptr_i];
                job_ptr_i += 8;
                // addressing_mode
                uint64_t is_pointer_access = *(uint64_t *)&job_ptr[job_ptr_i];
                job_ptr_i += 8;

                arrays.emplace_back(array_id, dst_id,
                    vaddr_start, vaddr_end, element_size,
                    is_ranged_access, is_pointer_access);
            }
        }
        std::string to_string() const
        {
            std::stringstream ss;
            for (const auto &array : arrays) {
                ss << array.to_string() << "\n";
            }
            return ss.str();
        }
}; // class PickleJobCollection

}; // namespace gem5

#endif // PICKLE_JOB_HH
