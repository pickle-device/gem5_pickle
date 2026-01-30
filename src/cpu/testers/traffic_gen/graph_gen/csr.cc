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

#include "cpu/testers/traffic_gen/graph_gen/csr.hh"

#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace gem5
{

CSR::CSR(const std::string &_graph, const bool _is_directed)
{
    // Construct the CSR representation from the input graph
    std::ifstream infile(_graph);
    std::string line;
    std::map<int, std::set<int>> adjacencyList;
    uint64_t max_vertex_id = 0;
    uint64_t num_edges = 0; // just an over-estimation
    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        int src, dest;
        if (line.empty() || line[0] == '#') { continue; } // skip comments
        if (!(iss >> src >> dest)) { continue; } // skip invalid lines
        adjacencyList[src].insert(dest);
        num_edges++;
        if (!_is_directed) {
            adjacencyList[dest].insert(src);
            num_edges++;
        }
        if (src > max_vertex_id) max_vertex_id = src;
        if (dest > max_vertex_id) max_vertex_id = dest;
    }
    // Build rowPtr and colIdx
    rowPtr = std::vector<int>(max_vertex_id + 2, 0);
    rowPtr[0] = 0;
    colIdx.reserve(num_edges);
    for (uint64_t src = 0; src <= max_vertex_id; ++src) {
        auto neighbors_it = adjacencyList.find(src);
        if (neighbors_it == adjacencyList.end()) {
            // No neighbors
            rowPtr[src + 1] = rowPtr[src];
            continue;
        }
        auto &neighbors = neighbors_it->second;
        rowPtr[src + 1] = rowPtr[src] + neighbors.size();
        colIdx.insert(colIdx.end(), neighbors.begin(), neighbors.end());
    }
}

uint64_t
CSR::getNumVertices() const
{
    return rowPtr.size() - 1;
}

uint64_t
CSR::getNumEdges() const
{
    return colIdx.size();
}

} // namespace gem5
