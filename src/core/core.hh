// o3 RISC simulator
//
// out-of-order core
// - main functions
// - decode
// - rename/alloc
//
// Lukas Heine 2021

#ifndef SIM_CORE_H
#define SIM_CORE_H

#include "cconf.hh"

#include "uops.hh"
#include "../types.hh"
#include "../util.hh"
#include "../sim.hh"
#include "../mem.hh"

#include "../frontend/frontend.hh"

// register with size of N bytes
template<u8 N>
class Register
{
    public:
    template<typename T> T    read();
    template<typename T> T    read(size_t offs);
    void                      read(void* data, size_t len);

    template<typename T> void write(T val);
    template<typename T> void write(T val, size_t offs);
    void                      write(void* data, size_t len);

    wN<N> content = { std::byte {0} };

    wN<N>& operator*() { return &this->content.b; }
}__attribute__((may_alias)); // Register

struct CoreException : public SimulatorException {};

struct InvalidRegAccessException : public CoreException
{
    const char* what() const throw()
    { return "invalid register access"; }
}; // InvalidRegAccessException

// https://patents.google.com/patent/US7765384B2 might use this
// visible ARF
struct ArchRegFile
{
    Register<REGCLS_0_SIZE> gp[REGCLS_0_CNT];
    Register<REGCLS_1_SIZE> fp[REGCLS_1_CNT];
    Register<REGCLS_2_SIZE> vr[REGCLS_2_CNT];

    Register<CCREG_SIZE>    cc;
    Register<ADDR_SIZE>     ip;
}; // ArchRegFile

// only visible to core
struct PhysRegFile
{
    Register<REGCLS_0_SIZE> gp[REGCLS_0_RNREG];
    Register<REGCLS_1_SIZE> fp[REGCLS_1_RNREG];
    Register<REGCLS_2_SIZE> vr[REGCLS_2_RNREG];
    Register<CCREG_SIZE>    cc[CCREG_CNT];
}; // PhysRegFile

struct RenameTable
{
    // archreg -> last allocated preg
    u8 gp[REGCLS_0_CNT];
    u8 fp[REGCLS_1_CNT];
    u8 vr[REGCLS_2_CNT];

    // commit table should be enough to save state, see e.g. BOOM

    // archreg -> last commited preg
    u8 gc[REGCLS_0_CNT];
    u8 fc[REGCLS_1_CNT];
    u8 vc[REGCLS_2_CNT];

    // preg -> archreg
    u8 pg[REGCLS_0_RNREG];
    u8 pf[REGCLS_1_RNREG];
    u8 rv[REGCLS_2_RNREG];

    // free lists for pregs
    std::deque<u8> gp_freelist; // unallocated physical gp regs
    std::deque<u8> fp_freelist; // unallocated physical fp regs
    std::deque<u8> vr_freelist; // unallocated physical vector regs
    std::deque<u8> cc_freelist; // usable condition registers
    std::deque<u8> cc_lastused; // last set condition registers
};

// stores will be controlled by the ROB
// loads can be executed speculatively

struct ROBEntry
{
    MM::MemoryRef mref;    // ld/st metadata
    uop           op;      // uop
    u64           c_ready; // ready to commit == val ready
    u32           except;  // exception occured with op
    u8            in_exec; // uop in execution
    u8            cc_use;  // used condition register
    u8            cc_set;  // set condition register
}; // ROBEntry

const ROBEntry zero_re = { MM::zero_mref, zero_op, 0, 0, 0, 0, 0 };

typedef enum
{
    commit_unavail,
    commit_ready
} commit_status;

typedef enum
{
    exec_waiting,
    exec_running,
} exec_status;

typedef enum
{
    fu_ready,
    fu_busy,
} fu_status;

struct FUInfo
{
    ROBEntry*   re;                     // associated ROB entry
    const u8    type;                   // fu_type
    u8          busy;                   // remaining cycles in execution
    const u8    id;
    u64         cycle;                  // cycle this FU will start execution

    FUInfo(u8 type, u8 id) : re(nullptr), type(type), busy(fu_ready), id(id), cycle(0) {};
}; // FUInfo

struct RSPort
{
    u8              id;
    u16             busy; // available / remaining latency
    vector<FUInfo>  fus;  // associated FUs at port

    RSPort(u8 id, vector<u8> types);
}; // RSPort

struct ReservationStation
{
    vector<RSPort> ports =
    {
        RSPort(0, {fu_alu, fu_div,  fu_brch, fu_ctrl }),
        RSPort(1, {fu_alu, fu_mul                    }),
        RSPort(2, {fu_alu, fu_agu                    }),
        RSPort(3, {fu_alu, fu_brch, fu_ctrl          }),
        RSPort(4, {fu_agu, fu_ld                     }),
        RSPort(5, {fu_agu, fu_ld                     }),
        RSPort(6, {fu_st                             }),
        RSPort(7, {fu_agu                            }),
    };
}; // ReservationStation

struct LoadQueueEntry
{
    ROBEntry*   re;         // associated ROB entry with memory reference
    u64         res_cycle;  // 
}; // LoadQueueEntry

struct StoreAddressEntry
{
    u64         vaddr;
    size_t      len;
}; // StoreQueueEntry

class Core
{
    public:
    Core(LatchQueue<uop>* uqueue, Simulator::SimulatorState& state, MemoryManager& mmu,
        Frontend& fe);
    ~Core();
    u32 cycle();
    u8  flush();

    u32 decode();
    u32 alloc();
    u32 issue();
    u32 execute();
    u32 commit();

    template<u8 N>
    u8              run_uop(ROBEntry& re, Register<N>* regfile);
    vector<RSPort*> get_rsports(const u8 portmask);
    inline void     set_cc(u8 reg, u64 cc) { prf.cc[reg].write<u64>(cc); };

    std::stringstream idra_readable(u8 n);
    std::stringstream rob_readable(u8 n);
    std::stringstream prf_readable(u8 regclass);

    private:
    LatchQueue<uop>*           uqueue;
    Simulator::SimulatorState& state;
    MemoryManager&             mmu;
    Frontend&                  fe;
    PhysRegFile                prf;
    RenameTable                rrt;
    ReservationStation         rs;
    LatchQueue<uop>*           id_ra;            // decode / rename&alloc
    LatchQueue<ROBEntry>*      rob;
    LatchQueue<ROBEntry*>*     ldq;              // load queue

    u64                        seq_at_alloc = 0; // index into seq_addrs
    u64                        rip_at_alloc = 0; // may not need this
    u16 next_inactive;                           // inactive next cycle (mask)
}; // Core

template<u8 N>
std::ostream& operator<<(std::ostream& os, const Register<N>& reg);
std::ostream& operator<<(std::ostream& os, const ROBEntry& re);
std::ostream& operator<<(std::ostream& os, const FUInfo& fui);
std::ostream& operator<<(std::ostream& os, const RSPort& rsp);
std::ostream& operator<<(std::ostream& os, const ReservationStation& rs);

#include "core.tt"

#endif // SIM_CORE_H
