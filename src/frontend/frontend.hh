// o3 RISC simulator
//
// frontends
//
// Lukas Heine 2021

#ifndef SIM_FRONTEND_H
#define SIM_FRONTEND_H

#include <deque>

#include "../util.hh"
#include "../sim.hh"
#include "../mem.hh"

#include "bp.hh"

class Frontend
{
    public:
    // frontend receives bytecode, does stuff, then places uops into the queue
    // todo replace bytecode with generic calls to MMU
    // Frontend(MemoryManager& mmu, ..)

    // Frontend(std::vector<u8>& bytecode, LatchQueue<struct uop>* uqueue,
    //         Simulator::SimulatorState& state) 
    //     : bytecode(bytecode), uqueue(uqueue), state(state) {};
    Frontend(MemoryManager& mmu, LatchQueue<uop>* uqueue, Simulator::SimulatorState& state)
        : mmu(mmu), uqueue(uqueue), state(state) {};
    virtual u8                cycle()   = 0;
    virtual u8                flush()   = 0;
    virtual std::stringstream summary() = 0;

    void       set_fetchaddr(u64 rip)   { fetchaddr = rip; };
    
    // overwrite this and then check at alloc to remove any load/exec stalls
    // need free and used list, get treg at alloc, discard after first read
    // virtual u8 is_tempreg(u8 reg)       { return reg & 0; };

    BranchPredictor*            bp;

    protected:
    u64                         fetchaddr;
    MemoryManager&              mmu;
    LatchQueue<uop>*            uqueue;
    Simulator::SimulatorState&  state;
};

class RiscFrontend : public Frontend
{
    public:
    RiscFrontend(MemoryManager& mmu, LatchQueue<uop>* uqueue,
            Simulator::SimulatorState& state);
    ~RiscFrontend();
    u8                cycle();
    u8                flush();
    std::stringstream summary();

};

#endif // SIM_FRONTEND_H
