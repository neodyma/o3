// o3 RISC simulator
// 
// Simulator definitions
//
// Lukas Heine 2021

#ifndef SIM_MAIN_H
#define SIM_MAIN_H

#include <deque>

#include "util.hh"
#include "types.hh"
#include "conf.hh"

// #include "core/cconf.hh"

class MemoryManager;
class Frontend;
class Core;
class ArchRegFile;

u8 map_elf();

typedef enum
{
    risc, x64,
} frontends;

typedef enum
{
    if_active   = 0x0001, // fetch
    pd_active   = 0x0002, // predecode
    de_active   = 0x0004, // macro decode

    id_active   = 0x0100, // uop decode
    ra_active   = 0x0200, // rename/alloc
    is_active   = 0x0400, // issue
    ex_active   = 0x0800, // execute/mem
    co_active   = 0x1000, // commit

    fe_active   = 0x0007, // entire frontend
    core_active = 0x1f00, // entire core
} pipeline_status;

typedef enum
{
    pl_kernel, pl_user = 3,
} protection_level;

namespace cpuid
{
    struct cpuid_regs
    {
        u32 eax, ebx, ecx, edx;
    }; // cpuid_regs

    // _r*: reserved, _z*: always zero
    typedef enum
    {
        fpu   = (1 <<  0), vme   = (1 <<  1), de    = (1 <<  2), pse   = (1 <<  3),
        rsc   = (1 <<  4), msr   = (1 <<  5), pae   = (1 <<  6), mce   = (1 <<  7),
        cx8   = (1 <<  8), apic  = (1 <<  9), d1_r0 = (1 << 10), sep   = (1 << 11),
        mtrr  = (1 << 12), pge   = (1 << 13), mca   = (1 << 14), cmov  = (1 << 15),
        pat   = (1 << 16), pse36 = (1 << 17), psn   = (1 << 18), clfsh = (1 << 19),
        d1_r1 = (1 << 20), ds    = (1 << 21), acpi  = (1 << 22), mmx   = (1 << 23),
        fxsr  = (1 << 24), sse   = (1 << 25), sse2  = (1 << 26), ss    = (1 << 27),
        htt   = (1 << 28), tm    = (1 << 29), d1_r2 = (1 << 30), pbe   = (1 << 31)
    } cpuid1_edx;

    typedef enum
    {
        sse3  = (1 <<  0), pclmulqdq  = (1 <<  1), dtes64 = (1 <<  2), monitor = (1 <<  3),
        dscpl = (1 <<  4), vmx        = (1 <<  5), smx    = (1 <<  6), eist    = (1 <<  7),
        tm2   = (1 <<  8), ssse3      = (1 <<  9), cnxtid = (1 << 10), sdbg    = (1 << 11),
        fma   = (1 << 12), cmpxchg16b = (1 << 13), xtpruc = (1 << 14), pdcm    = (1 << 15),
        c1_r0 = (1 << 16), pcid       = (1 << 17), dca    = (1 << 18), sse41   = (1 << 19),
        sse42 = (1 << 20), x2apic     = (1 << 21), movbe  = (1 << 22), popcnt  = (1 << 23),
        tscd  = (1 << 24), aes        = (1 << 25), xsave  = (1 << 26), osxsave = (1 << 27),
        avx   = (1 << 28), f16c       = (1 << 29), rdrand = (1 << 30), c1_z0   = (1 << 31)
    } cpuid1_ecx;

    void cpuid(cpuid_regs& cr, u64 rax);
}; // cpuid

class Simulator
{
    public:
    Simulator(opts& myopts);
    u16 cycle();

    // cpuid::cpuid_regs cpuid;

    struct SimulatorState
    {   // todo sort
        u64 cycle;                    // current cycle
        u16 active;                   // pipeline status
        i8  ring;                     // current protection level
        
        std::deque<u64> in_flight;    // in flight instructions, plus predicted after the last
        std::deque<u64> seq_addrs;    // subsequent instruction pointers for each in flight op

        u64 refetch_at;               // refetch if uop at this vaddr tries to commit
        u8  refetch_active;           // refetch enabled
        u64 exception;                // exception status
        u64 commited_micro;
        u64 commited_macro;
        u64 flushes;

        // std::map<u16, u64> used_uops;

        // todo
        // - locks
        // - more events

        ArchRegFile*            arf;
        std::stringstream       arf_readable(u8 reglass);
        std::stringstream       state_readable(u8 max);
    } state; // SimulatorState

    LatchQueue<uop>*    uqueue;
    MemoryManager*      mmu;
    Frontend*           frontend;
    Core*               core;

    u8*                 stack;
}; // Simulator

struct NotImplementedException : public SimulatorException
{
    const char* what () const throw ()
    {   return "not implemented."; }
}; // NotImplementedException

struct InvalidArgumentException : public SimulatorException
{
    const char* what () const throw ()
    {   return "invalid function argument(s)."; }
}; // InvalidArgumentException

template<u8 N>
std::ostream operator<<(std::ostream& os, const ArchRegFile& arf);

#endif