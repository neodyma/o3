// o3 RISC simulator
//
// main functions and startup
// - read arguments
// - prepare memory, simulator
// - start pipeline
// - get results
//
// Lukas Heine 2021

#ifdef simtest
#include <gtest/gtest.h>
#endif // simtest

#include "sim.hh"
#include "types.hh"
#include "util.hh"

#include "mem.hh"
#include "frontend/frontend.hh"
#include "frontend/x64.hh"
#include "core/core.hh"

u8 loglevel;

Simulator::Simulator(opts& myopts)
{
    if(myopts.frontend != x64 && (myopts.code.size() % 16))
        util::abort("Machine code length is not a multiple of 16 bytes.");

    state = 
    {   
        0,                         // cycle
        (fe_active | core_active), // active
        pl_user,                   // ring
        { MM_USER_START }, { },    // cur / seq
        0, 0,                      // refetch ip, enabled
        ex_NONE,                   // exception
        0, 0, 0,                   // events
        nullptr                    // arf
    };

    mmu       = new MemoryManager(state);
    uqueue    = new LatchQueue<uop>(UQUEUE_SIZE + FETCH_WIDTH);
    state.arf = new ArchRegFile();
    
    switch(myopts.frontend)
    {
        case x64:
            frontend = new x64Frontend(*mmu, uqueue, state);
            state.arf->gp[to_ureg(reg64_sp)].write<u64>(STACK_START + STACK_SIZE); // init stack pointer
            break;

        default:
            frontend = new RiscFrontend(*mmu, uqueue, state);   
            // todo register convention?
            break;
    }
    core      = new Core(uqueue, state, *mmu, *frontend);

    state.arf->cc.write<u64>(0);
    state.arf->ip.write<u64>(MM_USER_START);
    frontend->set_fetchaddr(MM_USER_START);

    {   // map entire code
        auto frames = mmu->mmap_frames(MM_USER_START, myopts.code.data(), myopts.code.size(), pl_user, (MM::p_r | MM::p_x),
            ".text");
    
        for(auto frame : frames)
            mmu->map_page(frame.second, frame.second, 1, pl_user, (MM::p_r | MM::p_x));
    }

    stack = (u8*) aligned_alloc(PAGE_SIZE, STACK_SIZE);
    for(u16 i = 0; i < STACK_SIZE; i++)
        stack[i] = (u8)i;

    {   // map stack
        auto frames = mmu->mmap_frames(STACK_START, stack, STACK_SIZE, pl_user, (MM::p_r | MM::p_w), ".data");
        for(auto frame : frames)
            mmu->map_page(frame.second, frame.second, 1, pl_user, (MM::p_r | MM::p_w));
    }
}

u16 Simulator::cycle()
{
    // util::log(LOG_STATE_PRE, state.state_readable(8).str());
    // util::log(LOG_STATE_PRE, frontend.state_readable(8).str());
    // util::log(LOG_STATE_PRE, core.state_readable(8).str());

    frontend->cycle();
    core->cycle(); 

    // util::log(LOG_STATE_POST, state.state_readable(8).str());

    // execute pending stores even when fe or core are inactive
    return state.active | mmu->active();
}

// set cpuid_regs depending on rax
void cpuid::cpuid(cpuid_regs& cr, u64 rax)
{
    asm("cpuid"
        : "=a" (cr.eax), "=b" (cr.ebx), "=c" (cr.ecx), "=d" (cr.edx)
        : "a" (rax));
}

std::stringstream Simulator::SimulatorState::arf_readable(u8 regclass)
{
    std::stringstream ss;

    if(regclass == 0) // gp
        for(u16 i = 0; i < REGCLS_0_CNT; i++)
            ss << "r" << dec_u<3> << i << " " << hex_u<REGCLS_0_SIZE*8>
               << arf->gp[i] << (i % 4 == 3 ? "\n" : " ");

    if(regclass == 1) // fp
        for(u16 i = 0; i < REGCLS_1_CNT; i++)
            ss << "r" << dec_u<3> << i << " " << hex_u<REGCLS_1_SIZE*8>
               << arf->fp[i] << (i % 2 == 1 ? "\n" : " ");

    if(regclass == 2) // vr
        for(u16 i = 0; i < REGCLS_2_CNT; i++)
            ss << "r" << dec_u<3> << i << " " << hex_u<REGCLS_2_SIZE*8>
               << arf->vr[i] << "\n";

    return ss;
}

std::stringstream Simulator::SimulatorState::state_readable(u8 max)
{
    std::stringstream ss;

    if(!in_flight.empty())
        ss << "\nIn flight instructions:\n"; 

    for(u8 i = 0; i < in_flight.size() && i < max; i++)
        ss << dec_u_lf<2> << +i << " |    " << hex_u<64> << in_flight.at(i) << "\n";

    if(!seq_addrs.empty())
        ss << "\nSequential instructions:\n"; 

    for(u8 i = 0; i < seq_addrs.size() && i < max; i++)
        ss << dec_u_lf<2> << +i << " |    " << hex_u<64> << seq_addrs.at(i) << "\n";

    return ss;
}

// filter architectural state
std::stringstream RiscFrontend::summary()
{
    std::stringstream ss; ss << "\n";

    ss << "ARF GP:\n" << state.arf_readable(0).str();
    ss << "cc:  " << hex_u<64> << state.arf->cc.read<u64>();

    ss << "\n";
    return ss;
}

// extract mapped x64 registers form the arf
std::stringstream x64Frontend::summary()
{
    std::stringstream ss; ss << "\n";

    for(u8 i = 0; i <= reg64_gsbase; i++)
        ss << std::setw(4) << std::left << std::setfill(' ') << x64gp_str[i] << " " << hex_u<REGCLS_0_SIZE*8>
           << state.arf->gp[to_ureg(i)].read<u64>() << (i % 4 == 3 ? "\n" : " ");

    ss << "rflags " << hex_u<64> << state.arf->cc.read<u64>();

    ss << "\n";
    return ss;
}


int main(int argc, char** argv)
{
    opts myopts;
    if(util::parseargs(argc, argv, &myopts)) util::abort("Parsing args failed.");

    // log args
    util::log(LOG_SIM_INIT, "Simulator started with args:" );
    util::log(LOG_SIM_INIT, "        loglevel:   ", +loglevel);  
    util::log(LOG_SIM_INIT, "        frontend:   ", ((myopts.frontend == x64) ? "x64" : "RISC"));
    util::log(LOG_SIM_INIT, "        max cycles: ", MAX_CYCLES, "\n");

    Simulator sim = Simulator(myopts);

    timespec start, end;
    if(myopts.time) clock_gettime(CLOCK_MONOTONIC_RAW, &start);
    for(;sim.state.cycle < MAX_CYCLES;)
    {
        sim.state.cycle++;
        util::log(1, H2LINE, "\nEntering cycle ", dec_u<0>, sim.state.cycle, ".");
        util::log(1, "RIP ", hex_u<64>, sim.state.arf->ip.read<u64>());
        if(!sim.cycle()) break;
    }
    if(myopts.time) clock_gettime(CLOCK_MONOTONIC_RAW, &end);

    util::log_always(H2LINE);
    util::log_always("Simulator exited after ", dec_u<0>, sim.state.cycle, " cycles ", "with rip ", hex_u<64>,
        sim.state.arf->ip.read<u64>(), ".");


    util::log_always("\n", HLINE, sim.frontend->summary().str(), HLINE, "\n");


    // move these to ::summary()?
    util::log_always("Committed uops: ", dec_u<0>, sim.state.commited_micro, ". IPC: ", 
        ((f32)sim.state.commited_micro / (f32)sim.state.cycle));
    util::log_always("Committed mops: ", dec_u<0>, sim.state.commited_macro, ". IPC: ", 
        ((f32)sim.state.commited_macro / (f32)sim.state.cycle));
    util::log_always("Flushes:        ", dec_u<0>, sim.state.flushes);

    if(sim.state.exception) util::log_always("Core exception: ", getExceptNum(sim.state.exception), " ",
        exception_str[getExceptNum(sim.state.exception)], ", EC ", hex_u<16>, getExceptEC(sim.state.exception), ".");

    if(myopts.time)
    {
        timespec res;
        util::timediff(&res, &start, &end);
        util::log_always("time ", res.tv_sec, ".", dec_u_lf<6>, res.tv_nsec/1000, "s");
    }

    // std::cout << "\n";
    // for(u16 i = 0; i < 512; i++)
    //     std::cout << hex_u<8> << +sim.stack[i] << (i % 32 == 31 ? "\n" : " ");

    util::log_always(H2LINE);
    return EXIT_SUCCESS;
}
