/*
 * Copyright (c) 1999-2012 Mark D. Hill and David A. Wood
 * All rights reserved.
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

/*
 * Contains all of the various parts of the system we are simulating.
 * Performs allocation, deallocation, and setup of all the major
 * components of the system
 */

#ifndef __MEM_RUBY_SYSTEM_RUBYSYSTEM_HH__
#define __MEM_RUBY_SYSTEM_RUBYSYSTEM_HH__

#include <unordered_map>

#include "base/callback.hh"
#include "base/output.hh"
#include "mem/packet.hh"
#include "mem/ruby/profiler/Profiler.hh"
#include "mem/ruby/slicc_interface/AbstractController.hh"
#include "mem/ruby/slicc_interface/ProtocolInfo.hh"
#include "mem/ruby/system/CacheRecorder.hh"
#include "params/RubySystem.hh"
#include "sim/clocked_object.hh"

namespace gem5
{

namespace memory
{
class SimpleMemory;
} // namespace memory

namespace ruby
{

class Network;
class AbstractController;

class RubySystem : public ClockedObject
{
  public:
    PARAMS(RubySystem);
    RubySystem(const Params &p);
    ~RubySystem();

    // config accessors
    int getRandomization() { return m_randomization; }
    uint32_t getBlockSizeBytes() { return m_block_size_bytes; }
    uint32_t getBlockSizeBits() { return m_block_size_bits; }
    uint32_t getMemorySizeBits() { return m_memory_size_bits; }
    bool getWarmupEnabled() { return m_warmup_enabled; }
    bool getCooldownEnabled() { return m_cooldown_enabled; }

    memory::SimpleMemory *getPhysMem() { return m_phys_mem; }
    Cycles getStartCycle() { return m_start_cycle; }
    bool getAccessBackingStore() { return m_access_backing_store; }

    // Public Methods
    Profiler*
    getProfiler()
    {
        assert(m_profiler != NULL);
        return m_profiler;
    }

    void regStats() override {
        ClockedObject::regStats();
    }
    void collateStats() { m_profiler->collateStats(); }
    void resetStats() override;

    void memWriteback() override;
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
    void drainResume() override;
    void process();
    void init() override;
    void startup() override;
    bool functionalRead(Packet *ptr);
    bool functionalWrite(Packet *ptr);

    void registerNetwork(Network*);
    void registerAbstractController(
        AbstractController*, std::unique_ptr<ProtocolInfo>
    );
    void registerMachineID(const MachineID& mach_id, Network* network);
    void registerRequestorIDs();

    bool eventQueueEmpty() { return eventq->empty(); }
    void enqueueRubyEvent(Tick tick)
    {
        auto e = new EventFunctionWrapper(
            [this]{ processRubyEvent(); }, "RubyEvent");
        schedule(e, tick);
    }

    const ProtocolInfo& getProtocolInfo() { return *protocolInfo; }

  private:
    // Private copy constructor and assignment operator
    RubySystem(const RubySystem& obj);
    RubySystem& operator=(const RubySystem& obj);

    void makeCacheRecorder(uint8_t *uncompressed_trace,
                           uint64_t cache_trace_size,
                           uint64_t block_size_bytes);

    static void readCompressedTrace(std::string filename,
                                    uint8_t *&raw_data,
                                    uint64_t &uncompressed_trace_size);
    static void writeCompressedTrace(uint8_t *raw_data, std::string file,
                                     uint64_t uncompressed_trace_size);

    void processRubyEvent();

    // Called from `functionalRead` depending on if the protocol needs
    // partial functional reads.
    bool simpleFunctionalRead(PacketPtr pkt);
    bool partialFunctionalRead(PacketPtr pkt);

  private:
    // configuration parameters
    bool m_randomization;
    uint32_t m_block_size_bytes;
    uint32_t m_block_size_bits;
    uint32_t m_memory_size_bits;

    bool m_warmup_enabled = false;
    bool m_cooldown_enabled = false;
    memory::SimpleMemory *m_phys_mem;
    const bool m_access_backing_store;

    //std::vector<Network *> m_networks;
    std::vector<std::unique_ptr<Network>> m_networks;
    std::vector<AbstractController *> m_abs_cntrl_vec;
    Cycles m_start_cycle;

    std::unordered_map<MachineID, unsigned> machineToNetwork;
    std::unordered_map<RequestorID, unsigned> requestorToNetwork;
    std::unordered_map<unsigned, std::vector<AbstractController*>> netCntrls;

    std::unique_ptr<ProtocolInfo> protocolInfo;

  public:
    Profiler* m_profiler;
    CacheRecorder* m_cache_recorder;
    std::unordered_map<MachineID, AbstractController*> \
      machineIDToControllerMap;
    std::vector<std::map<uint32_t, AbstractController *> > m_abstract_controls;
    std::map<MachineType, uint32_t> m_num_controllers;

    // These are auto-generated by SLICC based on the built protocol.
    int MachineType_base_count(const MachineType& obj);
    int MachineType_base_number(const MachineType& obj);
};

} // namespace ruby
} // namespace gem5

#endif //__MEM_RUBY_SYSTEM_RUBYSYSTEM_HH__
