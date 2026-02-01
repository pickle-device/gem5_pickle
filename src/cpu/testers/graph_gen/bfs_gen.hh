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

#ifndef __BFS_GEN_HH__
#define __BFS_GEN_HH__

#include <cstdint>
#include <memory>
#include <optional>
#include <queue>
#include <vector>

#include "base/logging.hh"
#include "base/types.hh"
#include "cpu/testers/graph_gen/csr.hh"
#include "debug/BFSGen.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "mem/request.hh"
#include "params/BFSGen.hh"
#include "sim/clocked_object.hh"
#include "sim/system.hh"

#define BFS_GEN_DEBUG(...) \
    DPRINTF(BFSGen, __VA_ARGS__)

namespace gem5
{

class GraphConstruct
{
  public:
    std::shared_ptr<CSR> csr;

    Addr work_queue_start_vaddr;
    unsigned work_queue_element_size;
    Addr work_queue_access_pc;

    Addr neighbor_ptr_start_vaddr;
    unsigned neighbor_ptr_element_size;
    Addr neighbor_ptr_access_pc;

    Addr neighbor_list_start_vaddr;
    unsigned neighbor_list_element_size;
    Addr neighbor_list_access_pc;

    Addr visited_list_start_vaddr;
    unsigned visited_list_element_size;
    Addr visited_list_access_pc;

  GraphConstruct(
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
  );
  void setCSR(std::shared_ptr<CSR> _csr);

  std::vector<bool> visited_list;

}; // class GraphConstruct

class DataAccess
{
  public:
    Addr address;
    Addr pc;
    uint64_t size;
    bool is_read;

    DataAccess(
      const Addr _address, const Addr _pc, const uint64_t _size,
      const bool _is_read
    );
}; // class DataAccess

// For each work item (travelling to a vertex's neighbors), we need to track
// the sequence of memory accesses that will be generated.
// We have 3 levels of depth:
// - Depth 1: Visiting the work queue to get the next vertex to visit
// - Depth 2: Visiting the neighbor pointers to get the start and end of
//            neighbor list for the current vertex
// - Depth 3: Visiting the neighbor list to get each neighbor vertex ID, and
//            for each neighbor, visiting the visited list to check if it has
//            been visited
class VisitorTracker
{
  public:
    VisitorTracker(
      const uint64_t _work_queue_index, const uint64_t _vertex_id,
      GraphConstruct &_graph
    );
    uint64_t getVertexId() const;
    bool isDone() const;
    // Get the next memory access (address and size) for this visitor
    std::optional<DataAccess> getNextAccesses();

  private:
    uint64_t work_queue_index;
    uint64_t vertex_id;
    GraphConstruct *graph;
    std::queue<DataAccess> expected_accesses;
}; // class VisitorTracker

class BFSGen : public ClockedObject
{
  private:
    class BFSGenPort : public RequestPort
    {
      private:
        BFSGen *owner;
      public:
        BFSGenPort(const std::string& name, BFSGen *owner);
        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;
    };

  public:
    BFSGen(const BFSGenParams &p);
    ~BFSGen();
    PARAMS(BFSGen);
    void startup() override;
    Port &getPort(
      const std::string &if_name, PortID idx=InvalidPortID
    ) override;
    void notifyPacketReceived(const Addr vaddr, const uint64_t vertex_id);

  private:
    System* system;
    std::shared_ptr<CSR> csr;
    BFSGenPort port;
    RequestorID requestorId;
    EventFunctionWrapper dataCheckEvent;
    EventFunctionWrapper sendPendingRequestEvent;
    EventFunctionWrapper visitorPromotionEvent;

  private:
    void sendFunctionalRead(Addr addr, uint8_t *data, unsigned size);
    void sendFunctionalWrite(Addr addr, const uint8_t *data, unsigned size);
    void addReadToPendingPackets(
      uint64_t vertex_id, Addr vaddr, Addr pc, uint64_t size
    );
    void addWriteToPendingPackets(
      uint64_t vertex_id, Addr vaddr, Addr pc, unsigned size,
      const uint8_t *data
    );
    void dataCheck();
    void scheduleDataCheckEvent();
    void sendPendingRequest();
    void scheduleSendPendingRequestEvent();
    bool visitorThreadsAvailable() const;
    void promoteVisitors();
    void scheduleVisitorPromotionEvent();
    void exitSimIfFinish() const;

  private:
    const uint64_t cache_block_size;
    const uint64_t source_vertex;
    const uint64_t num_visitor_threads;
    uint64_t max_num_responses;

    std::vector<uint64_t> work_queue;

    GraphConstruct graph;
    std::vector<VisitorTracker> visitor_trackers;
    uint64_t current_work_queue_index;

    struct BFSGenStats: public statistics::Group
    {
        BFSGen* owner;

        statistics::Scalar numResponsesReceived;

        BFSGenStats(BFSGen* _owner);
    } stats;

  public:
    // We share the following data structures with BFSGenPort
    // Inflight packets
    // Here, we track the vaddr and corresponding vertex IDs
    std::unordered_map<Addr, std::vector<uint64_t>> inflight_packets;
    // Pending packets that have yet to be sent out
    // Each entry is a pair of <packet, vertex_id>
    std::queue<std::pair<PacketPtr, uint64_t>> pending_packets;
}; // class BFSGen

} // namespace gem5

#endif // __BFS_GEN_HH__
