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

#include "cpu/testers/graph_gen/bfs_gen.hh"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <numeric>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "base/logging.hh"
#include "debug/BFSGen.hh"
#include "sim/eventq.hh"
#include "sim/sim_exit.hh"
#include "sim/system.hh"

namespace gem5
{

GraphConstruct::GraphConstruct(
    Addr work_queue_start_vaddr,
    unsigned work_queue_element_size,
    Addr work_queue_access_pc,
    Addr neighbor_ptr_start_vaddr,
    unsigned neighbor_ptr_element_size,
    Addr neighbor_ptr_access_pc,
    Addr neighbor_list_start_vaddr,
    unsigned neighbor_list_element_size,
    Addr neighbor_list_access_pc,
    Addr visited_list_start_vaddr,
    unsigned visited_list_element_size,
    Addr visited_list_access_pc
) : work_queue_start_vaddr(work_queue_start_vaddr),
    work_queue_element_size(work_queue_element_size),
    work_queue_access_pc(work_queue_access_pc),
    neighbor_ptr_start_vaddr(neighbor_ptr_start_vaddr),
    neighbor_ptr_element_size(neighbor_ptr_element_size),
    neighbor_ptr_access_pc(neighbor_ptr_access_pc),
    neighbor_list_start_vaddr(neighbor_list_start_vaddr),
    neighbor_list_element_size(neighbor_list_element_size),
    neighbor_list_access_pc(neighbor_list_access_pc),
    visited_list_start_vaddr(visited_list_start_vaddr),
    visited_list_element_size(visited_list_element_size),
    visited_list_access_pc(visited_list_access_pc)
{}

void
GraphConstruct::setCSR(std::shared_ptr<CSR> _csr)
{
    csr = _csr;
    // Initialize visited list
    visited_list = std::vector<bool>(csr->getNumVertices(), false);
}

DataAccess::DataAccess(
    const Addr _address, const Addr _pc, const uint64_t _size,
    const bool _is_read
) : address(_address), pc(_pc), size(_size), is_read(_is_read)
{}

VisitorTracker::VisitorTracker(
    const uint64_t _work_queue_index, const uint64_t _vertex_id,
    GraphConstruct &_graph
)
    : work_queue_index(_work_queue_index),
      vertex_id(_vertex_id),
      graph(&_graph)
{
    // Generate the expected memory access sequence for this visitor
    // 1. Access work queue to get vertex ID (already have it)
    const Addr work_queue_addr = graph->work_queue_start_vaddr +
        work_queue_index * graph->work_queue_element_size;
    expected_accesses.push(
        DataAccess(
          work_queue_addr, graph->work_queue_access_pc,
          graph->work_queue_element_size, true
        )
    );

    // 2. Access neighbor pointers to get start and end of neighbor list
    const Addr neighbor_ptr_addr = graph->neighbor_ptr_start_vaddr +
        vertex_id * graph->neighbor_ptr_element_size;
    expected_accesses.push(
        DataAccess(
          neighbor_ptr_addr, graph->neighbor_ptr_access_pc,
          graph->neighbor_ptr_element_size, true
        )
    );
    expected_accesses.push(
        DataAccess(
            neighbor_ptr_addr + graph->neighbor_ptr_element_size,
            graph->neighbor_ptr_access_pc, graph->neighbor_ptr_element_size,
            true
        )
    );

    // For simplicity, we assume we need to access the neighbor list, and
    // for each neighbor, access the visited list twice (one for read, one for
    // write).
    const uint64_t num_neighbors = \
      graph->csr->rowPtr[vertex_id + 1] - graph->csr->rowPtr[vertex_id];
    uint64_t first_neighbor_index = graph->csr->rowPtr[vertex_id];

    // 3. Access neighbor list to get each neighbor vertex ID
    for (uint64_t i = 0; i < num_neighbors; ++i) {
        const uint64_t neighbor_index = first_neighbor_index + i;
        const Addr neighbor_list_addr = graph->neighbor_list_start_vaddr +
            neighbor_index * graph->neighbor_list_element_size;
        expected_accesses.push(
            DataAccess(
              neighbor_list_addr, graph->neighbor_list_access_pc,
              graph->neighbor_list_element_size, true
            )
        );

        // 4. For each neighbor, access visited list to check if visited
        const uint64_t neighbor_vertex_id = graph->csr->colIdx[neighbor_index];
        const Addr visited_list_addr = graph->visited_list_start_vaddr +
            neighbor_vertex_id * graph->visited_list_element_size;
        expected_accesses.push(
            DataAccess(
              visited_list_addr, graph->visited_list_access_pc,
              graph->visited_list_element_size, true
            )
        );
        if (!graph->visited_list[neighbor_vertex_id]) {
            // If not visited, we will write to mark it as visited
            expected_accesses.push(
                DataAccess(
                  visited_list_addr, graph->visited_list_access_pc,
                  graph->visited_list_element_size, false
                )
            );
            // This is not precise timing, but for simplicity, we mark it as
            // visited here
            graph->visited_list[neighbor_vertex_id] = true;
        }
    }
}

uint64_t
VisitorTracker::getVertexId() const
{
    return vertex_id;
}

bool
VisitorTracker::isDone() const
{
    return expected_accesses.empty();
}

std::optional<DataAccess>
VisitorTracker::getNextAccesses()
{
    if (expected_accesses.empty()) {
        return std::nullopt;
    }

    DataAccess next_access = expected_accesses.front();
    expected_accesses.pop();
    return next_access;
}


BFSGen::BFSGenPort::BFSGenPort(const std::string &name, BFSGen *owner)
    : RequestPort(name), owner(owner)
{
}

bool
BFSGen::BFSGenPort::recvTimingResp(PacketPtr pkt)
{
    bool found = false;

    // Find the corresponding vertex ID from inflight packets
    for (auto it = owner->inflight_packets.begin();
         it != owner->inflight_packets.end(); ++it) {
        if (it->first == pkt->getAddr()) {
            found = true;
            const uint8_t* pkt_data_ptr = pkt->getConstPtr<uint8_t>();
            uint64_t pkt_data = 0;
            for (unsigned i = 0; i < pkt->req->getSize(); ++i) {
                pkt_data |= static_cast<uint64_t>(pkt_data_ptr[i]) << (i*8);
            }
            for (const auto &vertex_id : it->second) {
                BFS_GEN_DEBUG(
                    "Received timing response for address %#x "
                    "corresponding to vertex ID %lu, data: %#x\n",
                    pkt->getAddr(), vertex_id, pkt_data
                );
                owner->notifyPacketReceived(pkt->getAddr(), vertex_id);
            }
            owner->inflight_packets.erase(it);
            break;
        }
    }
    owner->stats.numResponsesReceived++;
    delete pkt;
    if (!found) {
        BFS_GEN_DEBUG(
            "Received timing response for unknown address %#x\n",
            pkt->getAddr()
        );
        return true;
    }
    return true;
}

void
BFSGen::BFSGenPort::recvReqRetry()
{
  if (owner->pending_packets.empty()) {
      return;
  }
  PacketPtr pkt = owner->pending_packets.front().first;
  uint64_t vertex_id = owner->pending_packets.front().second;
  if (sendTimingReq(pkt)) {
    owner->pending_packets.pop();
    if (owner->inflight_packets.find(pkt->getAddr()) ==
        owner->inflight_packets.end()) {
        owner->inflight_packets.emplace(
            std::make_pair(pkt->getAddr(), std::vector<uint64_t>())
        );
    }
    owner->inflight_packets[pkt->getAddr()].push_back(vertex_id);
  }
}

BFSGen::BFSGen(const BFSGenParams &p)
    : ClockedObject(p),
      system(p.system),
      port(name() + ".port", this),
      requestorId(p.system->getRequestorId(this)),
      dataCheckEvent(
        [this] { dataCheck(); }, name() + ".data_check_event"
      ),
      sendPendingRequestEvent(
        [this]() { sendPendingRequest(); },
        name() + ".send_pending_request_event"
      ),
      visitorPromotionEvent(
          [this]() { promoteVisitors(); },
          name() + ".visitor_promotion_event"
      ),
      cache_block_size(p.cache_block_size),
      source_vertex(p.source_vertex),
      num_visitor_threads(p.num_visitor_threads),
      max_num_responses(p.max_num_responses),
      graph(
          p.work_queue_start_vaddr,
          p.work_queue_element_size,
          p.work_queue_access_pc,
          p.neighbor_ptr_start_vaddr,
          p.neighbor_ptr_element_size,
          p.neighbor_ptr_access_pc,
          p.neighbor_list_start_vaddr,
          p.neighbor_list_element_size,
          p.neighbor_list_access_pc,
          p.visited_list_start_vaddr,
          p.visited_list_element_size,
          p.visited_list_access_pc
      ),
      current_work_queue_index(0),
      stats(this)
{
    BFS_GEN_DEBUG(
      "BFSGen started up with cache block size: %lu\n", cache_block_size
    );

    if (num_visitor_threads == 0) {
        fatal("Number of visitor threads must be greater than 0\n");
    }

    if (max_num_responses == 0) {
        max_num_responses = UINT64_MAX;
    } else {
        inform(
            "Maximum number of responses to process: %lu\n",
            max_num_responses
        );
    }
}

BFSGen::~BFSGen()
{
}

void
BFSGen::startup()
{
    // Load the graph in CSR format
    csr = std::make_shared<CSR>(params().graph_file, params().is_directed);
    graph.setCSR(csr);
    uint64_t num_vertices = csr->getNumVertices();
    uint64_t num_edges = csr->getNumEdges();
    BFS_GEN_DEBUG(
        "Loaded graph with %lu vertices and %lu edges\n",
        num_vertices, num_edges
    );
    BFS_GEN_DEBUG("Source vertex for BFS traversal: %lu\n", source_vertex);

    // Generate the BFS traversal memory access pattern here
    std::vector<bool> visited(num_vertices, false);
    work_queue.reserve(num_vertices);
    work_queue.push_back(source_vertex);
    visited[source_vertex] = true;
    uint64_t w = 0;
    while (w < work_queue.size()) {
        uint64_t current_vertex = work_queue[w];
        BFS_GEN_DEBUG("Visiting vertex %lu\n", current_vertex);
        // Get neighbors from CSR
        uint64_t row_start = csr->rowPtr[current_vertex];
        uint64_t row_end = csr->rowPtr[current_vertex + 1];
        for (uint64_t idx = row_start; idx < row_end; ++idx) {
            uint64_t neighbor = csr->colIdx[idx];
            // If neighbor not visited, add to work queue
            if (!visited[neighbor]) {
                visited[neighbor] = true;
                work_queue.push_back(neighbor);
                BFS_GEN_DEBUG(
                  "Found unvisited neighbor %lu, adding to work queue\n",
                  neighbor
                );
            }
        }
        w++;
    }
    BFS_GEN_DEBUG(
        "BFS traversal completed. Total vertices visited: %lu\n",
        work_queue.size()
    );

    // Now we dump the work queue, neighbor pointers, neighbor lists,
    // and visited list to memory
    // Dump work queue
    for (uint64_t i = 0; i < work_queue.size(); ++i) {
        Addr vaddr = graph.work_queue_start_vaddr +
            i * graph.work_queue_element_size;
        uint64_t vertex_id = work_queue[i];
        sendFunctionalWrite(
            vaddr, reinterpret_cast<uint8_t*>(&vertex_id),
            graph.work_queue_element_size
        );
        //const uint8_t* data_ptr =
        //    reinterpret_cast<const uint8_t*>(&vertex_id);
        //system->physProxy.writeBlob(
        //    vaddr, data_ptr, graph.work_queue_element_size
        //);
        BFS_GEN_DEBUG(
            "Wrote work queue entry %lu (vertex ID %lu) to address %#x\n",
            i, vertex_id, vaddr
        );
    }

    // Dump neighbor pointers
    for (uint64_t v = 0; v < num_vertices + 1; ++v) {
        Addr start_ptr_vaddr = graph.neighbor_ptr_start_vaddr +
            v * graph.neighbor_ptr_element_size;
        uint64_t data = csr->rowPtr[v];
        sendFunctionalWrite(
            start_ptr_vaddr, reinterpret_cast<uint8_t*>(&data),
            graph.neighbor_ptr_element_size
        );
        BFS_GEN_DEBUG(
            "Wrote neighbor pointer for vertex %lu (value %lu) to address %#x"
            "\n",
            v, data, start_ptr_vaddr
        );
    }

    // Dump neighbor lists
    for (uint64_t e = 0; e < num_edges; ++e) {
        Addr neighbor_list_vaddr = graph.neighbor_list_start_vaddr +
            e * graph.neighbor_list_element_size;
        uint64_t neighbor_vertex_id = csr->colIdx[e];
        sendFunctionalWrite(
            neighbor_list_vaddr,
            reinterpret_cast<uint8_t*>(&neighbor_vertex_id),
            graph.neighbor_list_element_size
        );
        BFS_GEN_DEBUG(
            "Wrote neighbor list entry %lu (neighbor vertex ID %lu) to "
            "address %#x\n",
            e, neighbor_vertex_id, neighbor_list_vaddr
        );
    }

    // Dump visited list (all initialized to false)
    for (uint64_t v = 0; v < num_vertices; ++v) {
        Addr visited_list_vaddr = graph.visited_list_start_vaddr +
            v * graph.visited_list_element_size;
        uint64_t visited_flag = 0; // false
        sendFunctionalWrite(
            visited_list_vaddr,
            reinterpret_cast<uint8_t*>(&visited_flag),
            graph.visited_list_element_size
        );
        BFS_GEN_DEBUG(
            "Wrote visited list entry for vertex %lu (value %d) to address %#x"
            "\n",
            v, visited_flag, visited_list_vaddr
        );
    }

    inform("BFS Source Vertex: %lu\n", source_vertex);
    inform("BFS Total Vertices Visited: %lu\n", work_queue.size());
    scheduleVisitorPromotionEvent();
}

Port&
BFSGen::getPort(const std::string &if_name, PortID idx)
{
    if (if_name != "port") {
        return ClockedObject::getPort(if_name, idx);
    }
    return port;
}

void
BFSGen::notifyPacketReceived(const Addr vaddr, const uint64_t vertex_id)
{
    BFS_GEN_DEBUG("Processing response for vertex ID %lu at address %#x\n",
                  vertex_id, vaddr);
    // Here, we can update any internal state or data structures
    // based on the received packet
    std::set<uint64_t> to_be_removed_vertex_ids;
    for (auto it = visitor_trackers.begin();
         it != visitor_trackers.end(); ++it) {
        if (it->getVertexId() == vertex_id) {
            // If this visitor is done, mark it for removal
            if (it->isDone()) {
                to_be_removed_vertex_ids.insert(vertex_id);
                continue;
            }
            // Process the next access for this visitor
            auto next_access_opt = it->getNextAccesses();
            if (next_access_opt.has_value()) {
                DataAccess next_access = next_access_opt.value();
                if (next_access.is_read) {
                    addReadToPendingPackets(
                        vertex_id, next_access.address, next_access.pc,
                        next_access.size
                    );
                } else {
                    // For write, we can send dummy data
                    std::vector<uint8_t> dummy_data(next_access.size, 0xFF);
                    addWriteToPendingPackets(
                        vertex_id, next_access.address, next_access.pc,
                        next_access.size, dummy_data.data()
                    );
                }
            }
        }
    }
    // Remove completed visitors
    visitor_trackers.erase(
        std::remove_if(
            visitor_trackers.begin(), visitor_trackers.end(),
            [&to_be_removed_vertex_ids](const VisitorTracker &vt) {
                return to_be_removed_vertex_ids.count(vt.getVertexId()) > 0;
            }
        ),
        visitor_trackers.end()
    );
    // Schedule adding visitors for new work queue entries
    scheduleVisitorPromotionEvent();
    // Schedule sending pending packets
    scheduleSendPendingRequestEvent();
    // If we receive all responses and have no more visitors, we can exit the
    // simulation
    exitSimIfFinish();
}

void
BFSGen::sendFunctionalRead(Addr addr, uint8_t *data, unsigned size)
{
    RequestPtr req = std::make_shared<Request>(addr, size, 0, requestorId);
    req->setPC(0xC0DE);
    PacketPtr pkt = new Packet(req, MemCmd::ReadReq);
    uint8_t* pkt_data = new uint8_t[req->getSize()];
    for (unsigned i = 0; i < size; ++i) {
        pkt_data[i] = 0;
    }
    pkt->dataDynamic(pkt_data);
    port.sendFunctional(pkt);
    std::memcpy(data, pkt_data, size);
    delete[] pkt_data;
    delete pkt;
}

void
BFSGen::sendFunctionalWrite(Addr addr, const uint8_t *data, unsigned size)
{
    RequestPtr req = std::make_shared<Request>(addr, size, 0, requestorId);
    req->setPC(0xC0DE);
    PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
    pkt->allocate();
    pkt->setData(data);
    port.sendFunctional(pkt);
    delete pkt;
}

void
BFSGen::addReadToPendingPackets(
    uint64_t vertex_id, Addr vaddr, Addr pc, uint64_t size
)
{
    RequestPtr req = std::make_shared<Request>(
        vaddr, size, 0, requestorId, pc, 0, nullptr
    );
    // Add a physical address to signal that the address is valid
    req->setPaddr(vaddr);
    PacketPtr pkt = new Packet(req, MemCmd::ReadReq);
    uint8_t* pkt_data = new uint8_t[req->getSize()];
    pkt->dataDynamic(pkt_data);
    pending_packets.push(std::make_pair(pkt, vertex_id));
    BFS_GEN_DEBUG(
        "Added read packet for vertex ID %lu at address %#x of size %lu to "
        "pending packets\n",
        vertex_id, vaddr, size
    );
}

void
BFSGen::addWriteToPendingPackets(
    uint64_t vertex_id, Addr vaddr, Addr pc, unsigned size, const uint8_t *data
)
{
    RequestPtr req = std::make_shared<Request>(
        vaddr, size, 0, requestorId, pc, 0, nullptr
    );
    // Add a physical address to signal that the address is valid
    req->setPaddr(vaddr);
    PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
    pkt->allocate();
    pkt->setData(data);
    pending_packets.push(std::make_pair(pkt, vertex_id));
    BFS_GEN_DEBUG(
        "Added write packet for vertex ID %lu at address %#x of size %lu to "
        "pending packets\n",
        vertex_id, vaddr, size
    );
}

void
BFSGen::dataCheck()
{
    // Check the work queue
    for (uint64_t i = 0; i < work_queue.size(); ++i) {
        Addr vaddr = graph.work_queue_start_vaddr +
            i * graph.work_queue_element_size;
        uint64_t vertex_id = 0;
        sendFunctionalRead(
            vaddr, reinterpret_cast<uint8_t*>(&vertex_id),
            graph.work_queue_element_size
        );
        BFS_GEN_DEBUG(
            "Checked work queue entry %lu at address %#x: vertex ID %lu;"
            " expected %lu\n",
            i, vaddr, vertex_id, work_queue[i]
        );
    }
}

void
BFSGen::scheduleDataCheckEvent()
{
    if (!dataCheckEvent.scheduled()) {
        schedule(dataCheckEvent, nextCycle());
    }
}

void
BFSGen::sendPendingRequest()
{
    while (!pending_packets.empty()) {
        PacketPtr pkt = pending_packets.front().first;
        uint64_t vertex_id = pending_packets.front().second;
        if (port.sendTimingReq(pkt)) {
            pending_packets.pop();
            if (inflight_packets.find(pkt->getAddr()) ==
                inflight_packets.end()) {
                inflight_packets.emplace(
                    std::make_pair(pkt->getAddr(), std::vector<uint64_t>())
                );
            }
            inflight_packets[pkt->getAddr()].push_back(vertex_id);
        } else {
            // Port is still blocked, exit the loop
            break;
        }
    }
}

void
BFSGen::scheduleSendPendingRequestEvent()
{
    if (!pending_packets.empty() && !sendPendingRequestEvent.scheduled()) {
        schedule(sendPendingRequestEvent, nextCycle());
    }
}

bool
BFSGen::visitorThreadsAvailable() const
{
    return visitor_trackers.size() < num_visitor_threads;
}

void
BFSGen::promoteVisitors()
{
    // Add new visitors from the work queue if there are available threads
    while (visitorThreadsAvailable() &&
           current_work_queue_index < work_queue.size()) {
        uint64_t vertex_id = work_queue[current_work_queue_index];
        visitor_trackers.emplace_back(
            current_work_queue_index, vertex_id, graph
        );
        BFS_GEN_DEBUG(
            "Promoted visitor for work queue index %lu (vertex ID %lu)\n",
            current_work_queue_index, vertex_id
        );
        current_work_queue_index++;

        // Start the first access for the new visitor
        auto next_access_opt = visitor_trackers.back().getNextAccesses();
        if (next_access_opt.has_value()) {
            DataAccess next_access = next_access_opt.value();
            if (next_access.is_read) {
                addReadToPendingPackets(
                    vertex_id, next_access.address, next_access.pc,
                    next_access.size
                );
            } else {
                // For write, we can send dummy data
                std::vector<uint8_t> dummy_data(next_access.size, 0xFF);
                addWriteToPendingPackets(
                    vertex_id, next_access.address, next_access.pc,
                    next_access.size, dummy_data.data()
                );
            }
        }
    }
    scheduleSendPendingRequestEvent();
}

void
BFSGen::scheduleVisitorPromotionEvent()
{
    if (
        current_work_queue_index < work_queue.size()
        && !visitorPromotionEvent.scheduled()
    ) {
        schedule(visitorPromotionEvent, nextCycle());
    }
}

void
BFSGen::exitSimIfFinish() const
{
    const bool current_work_queue_exhausted =
        current_work_queue_index >= work_queue.size();
    const bool no_active_visitors = visitor_trackers.empty();
    const bool no_pending_packets = pending_packets.empty();
    const bool max_responses_reached =
        stats.numResponsesReceived.value() >= max_num_responses;
    const bool no_inflight_packets = inflight_packets.empty();
    BFS_GEN_DEBUG(
        "Exit check: work queue exhausted: %d, no active visitors: %d, "
        "no pending packets: %d, max responses reached: %d, "
        "no inflight packets: %d\n",
        current_work_queue_exhausted,
        no_active_visitors,
        no_pending_packets,
        max_responses_reached,
        no_inflight_packets
    );
    if (
        current_work_queue_exhausted
        && no_active_visitors
        && no_pending_packets
        // There still might be inflight packets (as the visitors are removed
        // before the inflight packets are marked for deleted), but since
        // there's no active visitors, we can consider the work done.
        // && no_inflight_packets
    ) {
        BFS_GEN_DEBUG("BFSGen completed all work, exiting sim loop.\n");
        std::string words_to_automagically_generate_a_normal_exit_event =
            "BFSGen completed all work.";
        exitSimLoop(words_to_automagically_generate_a_normal_exit_event);
    }

    if (max_responses_reached) {
        BFS_GEN_DEBUG(
            "BFSGen reached maximum number of responses to process (%lu), "
            "exiting sim loop.\n",
            max_num_responses
        );
        std::string other_words_to_automagically_generate_a_normal_exit_event =
            "BFSGen reached maximum number of responses to process.";
        exitSimLoop(other_words_to_automagically_generate_a_normal_exit_event);
    }
}

BFSGen::BFSGenStats::BFSGenStats(BFSGen* _owner)
  : statistics::Group(_owner),
    owner(_owner),
    ADD_STAT(numResponsesReceived, statistics::units::Count::get(),
        "Number of responses received from memory.")
{
}

} // namespace gem5
