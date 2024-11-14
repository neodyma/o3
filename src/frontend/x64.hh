// o3 RISC simulator
// 
// x64 frontend
// - structures
// - instruction bytes
//
// Lukas Heine 2021

#ifndef SIM_FRONTEND64_H
#define SIM_FRONTEND64_H

#include "frontend.hh"
#include "x64-tables.hh"

#include "../core/uops.hh"

// decode metadata
struct x64d_meta
{
    u8 has_g1   = 0; // active group 1 prefix
    u8 has_g2   = 0; // active group 2 prefix
    u8 has_66   = 0;
    u8 has_67   = 0;
    u8 has_rex  = 0;
    u8 off_rex  = 0;
    u8 op_mode  = 0; // # of opcode escapes
    u8 decoder  = 0;
};

struct x64op
{
    vector<u8> bytes;
    // prefix always starts at offset 0 if present
    u8 off_opcode = 0; // these are offsets into .bytes
    u8 off_modrm  = 0; //
    u8 off_sib    = 0; //
    u8 off_displ  = 0; //
    u8 off_imm    = 0; //
    u8 len        = 0; // 0 invalid, >15 invalid -> #UD
    x64d_meta meta;
    // todo recognize partial instructions
};

typedef enum
{
    x64d_fast,  // simple decoder:    1 uop
    x64d_cmplx, // complex decoder:   <=4
    x64d_seq,   // uop sequencer/ROM: >4 uops (4 / cycle)
} x64d_type;

const x64op zero_x64op = { {}, 0, 0, 0, 0, 0, 0, {} };

std::ostream& operator<<(std::ostream& os, const x64op& op);


// TODO skip but recognize other segments
// TODO SSE prefix order
constexpr u8 is_legacy(u8 b)
{
    switch(b)
    {
        default: return 0;
        case 0xf0: case 0xf2: case 0xf3: // group 1: lock, repn/bnd, rep
        case 0x64: case 0x65:            // group 2: fs, gs (other segments ud or ignored?)
        case 0x66:                       // group 3: operand size
        case 0x67:                       // group 4: address size
            return 1;
    }
}

constexpr u8 is_esc1(const u8 b)  { return (b == 0x0f); }              // first escape byte
constexpr u8 is_esc2(const u8 b)  { return (b == 0x38 || b == 0x3a); } // second escape byte
constexpr u8 is_rex(const u8 b)   { return (b >= 0x40 && b <  0x50); } // rex prefix
constexpr u8 is_vex(const u8 b)   { return (b == 0xc4 || b == 0xc5); } // vex marker
constexpr u8 is_evex(const u8 b)  { return (b == 0x62); }              // evex marker

namespace modrm
{   
    typedef enum
    {   // mmrrr..
        rm  = 0x07, // rm
        reg = 0x38, // reg
        mod = 0xc0, // mod
    } modrm_bits;

    constexpr u8 get_rm(const u8 byte)  { return (byte & rm)  >> 0; }
    constexpr u8 get_reg(const u8 byte) { return (byte & reg) >> 3; }
    constexpr u8 get_mod(const u8 byte) { return (byte & mod) >> 6; }
} // modrm

namespace sib
{   
    typedef enum
    {   // ssiiibbb
        b = 0x07, // base
        i = 0x38, // index
        s = 0xc0, // scale
    } sib_bits;

    constexpr u8 get_b(const u8 byte) { return (byte & b) >> 0; }
    constexpr u8 get_i(const u8 byte) { return (byte & i) >> 3; }
    constexpr u8 get_s(const u8 byte) { return (byte & s) >> 6; }
} // sib

namespace rex
{
    // first nibble is always 0100
    typedef enum
    {
        b = 0x1, // rex.b
        x = 0x2, // rex.x
        r = 0x4, // rex.r
        w = 0x8, // rex.w
    } rex_bits;
} // rex

namespace vex
{
    // zeroth vex byte is c4 (3 bytes total):
    typedef enum
    { // first byte
        v3_m = 0x1f, // m
        v3_B = 0x20, // B
        v3_X = 0x40, // X
        v3_R = 0x80, // R
    } vex3_bits1;

    typedef enum
    { // second byte
        v3_p = 0x03, // pp
        v3_L = 0x04, // L
        v3_v = 0x78, // v
        v3_W = 0x80, // W
    } vex3_bits2;

    // zeroth vex byte is c5 (2 bytes total):
    // can merge with 3 byte field #2
    typedef enum
    { // first byte
        v2_p = 0x03, // pp
        v2_L = 0x04, // L
        v2_v = 0x78, // v
        v2_R = 0x80, // R
    } vex2_bits1;
} // vex

namespace evex
{
    // todo sometime
} // evex

// mode: 0 = 1 byte opcode, 1 = 2 byte opcode
// TODO somehow tell if modrm.mod is required as well
constexpr u8 get_group(const u8 byte, const u8 mode)
{
    switch(mode)
    {
        default: return 0;
        case 0:  return x64def::opgrp_1b(byte);
        case 1:  return x64def::opgrp_2b(byte);
    }  
}

// instruction may have a mandatory prefix
constexpr u8 has_reqpfx(const u8 byte, const u8 mode)
{
    switch(mode)
    {
        default: return 0;
        case 0:  return x64def::reqpfx_1b(byte);
        case 1:  return x64def::reqpfx_2b[byte];
    }
}

// mode: 0 = 1 byte opcode, 1 = 2 byte opcode
constexpr u8 use_modrm(const u8 byte, const u8 mode)
{
    switch(mode)
    {
        default: return 0;
        case 0:  return x64def::modrm_1b[byte];
        case 1:  return x64def::modrm_2b[byte];
    }
}

// does the instruction contain sib?
// sib is needed when mod =/= 11b and r/m = 100b 
constexpr u8 use_sib(const u8 modrm)
{
    return (modrm::get_mod(modrm) != 0b11 &&
            modrm::get_rm(modrm) == 0b100);
}

// displacement size in bytes
// controlled by modrm and addressing mode
// long mode will always use table 2-2 (addrsz == 4) so this is irrelevant
// sib.b == 0b101 also uses a displacement!
// sib * needs some work
constexpr u8 get_displsz(const u8 modrm, const u8 sib)
{
    if(use_sib(modrm) && sib::get_b(sib) == 0b101)
        switch(modrm::get_mod(modrm))
        {
            case 0b00: case 0b10:
                return 4;
            case 0b01:
                return 1;
            default:
                return 0;
        }
    else
        switch(modrm::get_mod(modrm))
        {
            case 0b00: return (modrm::get_rm(modrm) == 0b101 ? 4 : 0);
            case 0b01: return 1;
            case 0b10: return 4;
            default:
            case 0b11: return 0;
        }
}

// operand size in bytes
// this should be used if the operand type is already known
constexpr u8 get_opsz(const u8 opsz, const u8 optype)
{
    switch(optype)
    {
        default:        return optype;
        case x64def::v: return opsz;
        case x64def::z: return (opsz == 2 ? 2 : 4);
        case x64def::f: return (opsz == 2 ? 2 : 8);
        // case ::o
    }
}

// immediate size in bytes
// depending on opcode byte(s) and operand size
// use in predecode
// operand size determined by REX.W + 66/67:
// !W + !66/67 -> op32, addr64
// !W +  66/67 -> op16, addr32
//  W + !66/67 -> op64, addr64
//  W +  66/67 -> op64, addr32
// mode: 0 = 1B, 1 = 2B opcode
// always wrap the lookup in get_opsz....
constexpr u8 get_immsz(const u8 byte, const u8 opsz, const u8 mode, const u8 mod_reg)
{
    if(!mode)
    {
        u8 immtype = [&]() {
            u8 group = x64def::opgrp_1b(byte);
            if(!group) return x64def::immsz_1b[byte];
            else
            {
                x64def::x64opinfo info = x64def::get_opinfo({byte, mod_reg});
                u8 tmp = 0;
                for(auto i : info.operands)
                    if(i.addr_mode == x64def::I)
                        tmp += get_opsz(opsz, i.operand_type); // e.g. Ib + Iw = 3
                return tmp;
            }
        }();
        return get_opsz(opsz, immtype);
    }
    else if(mode == 1)
        return get_opsz(opsz, x64def::immsz_2b(byte));
        // TODO two byte groups
        // .. x64def::get_opinfo({0x0f, byte, mod_reg})
    else
        return 0;
}

// instruction excodes a branch
inline u8 is_branch(const x64op& op)
{ 
    // main opcode
    u8 opcode = op.bytes[op.off_opcode + op.meta.op_mode];
    
    switch(op.meta.op_mode)
    {
        default:
        case 0:
            switch(opcode)
            {
                case 0x70 ... 0x7f:
                case 0xca ... 0xcb:
                    return branch_cond;
                case 0xc2 ... 0xc3: // ret will always jump
                case 0xe8 ... 0xe9: // and so will call
                case 0xeb:
                    return branch_uncond;
                case 0xff:
                {
                    u8 modreg = modrm::get_reg(op.bytes[op.off_modrm]);
                    return (modreg >= 0b010 && modreg <= 0b101) ? branch_uncond : branch_none;
                }

                default:
                    return branch_none;
            }
            break;
        case 1:
            switch(opcode)
            {
                case 0x80 ... 0x8f:
                    return branch_cond;
                default:
                    return branch_none;
            }
    }
}

// instruction is a gp instruction
inline u8 is_gp(const x64op& op)
{
    u8 opcode = op.bytes[op.off_opcode + op.meta.op_mode];

    switch(op.meta.op_mode)
    {
        default:
        case 0:
            return 1;
        case 1:
            switch(opcode)
            {
                default:
                    return 0;
                case 0x05: case 0x07 ... 0x0a: case 0x0d:
                case 0x19: case 0x1c ... 0x1f:
                case 0x31:
                case 0x40 ... 0x4f:
                case 0x80 ... 0x9f:
                case 0xa0 ... 0xa5: case 0xa8 ... 0xa9: case 0xab ... 0xad: case 0xaf:
                case 0xb0 ... 0xbf:
                case 0xc0 ... 0xc1: case 0xc8 ... 0xcf:
                case 0xff:
                    return 1;
            }
    }
}

typedef enum
{
    ex64_DE  = 0x00, // divide error
    ex64_DB  = 0x01, // debug
    ex64_NMI = 0x02, // NMI
    ex64_BP  = 0x03, // breakpoint
    ex64_OF  = 0x04, // overflow
    ex64_BR  = 0x05, // bound range exc
    ex64_UD  = 0x06, // undefined opcode
    ex64_NM  = 0x07, // no math
    ex64_DF  = 0x08, // double fault
    ex64_TS  = 0x0a, // invalid TSS
    ex64_NP  = 0x0b, // segment not present
    ex64_SS  = 0x0c, // stack segment fault
    ex64_GP  = 0x0d, // general protection
    ex64_PF  = 0x0e, // page fault
    ex64_MF  = 0x10, // math fault
    ex64_AC  = 0x11, // alignment check
    ex64_MC  = 0x12, // machine check
    ex64_XM  = 0x13, // SIMD FP exception
    ex64_VE  = 0x14, // virtualization exc
    ex64_CP  = 0x15, // control protection
} x64_exceptions;

// x64 regs
// r0 is always zero and never writable!
typedef enum
{
    reg64_a,   // /al
    reg64_c,   // /cl
    reg64_d,   // /dl
    reg64_b,   // /bl
    reg64_sp,  // /ah
    reg64_bp,  // /ch
    reg64_si,  // /dh
    reg64_di,  // /bh
    reg64_r8,
    reg64_r9,
    reg64_r10,
    reg64_r11,
    reg64_r12,
    reg64_r13,
    reg64_r14,
    reg64_r15,
    reg64_fsbase,
    reg64_gsbase,

    // no control, debug, .. for now
    // plus some temp registers for loads
    // manage those?
    // - ring buffer and hope register pool is large enough
    // - assign temp registers on demand in IDRA
    // - ¯\_(ツ)_/¯


    // reset those?
    reg64_t0,
    reg64_tmax = 33,
} x64gp;

const std::string x64gp_str[reg64_gsbase + 1] =
{
    ("rax"), ("rcx"), ("rdx"), ("rbx"), ("rsp"), ("rbp"), ("rsi"), ("rdi"),
    ("r8"),  ("r9"),  ("r10"), ("r11"), ("r12"), ("r13"), ("r14"), ("r15"), 
    ("fsbase"), ("gsbase") 
};

typedef enum
{
    reg64_xmm0,
    reg64_xmm1,
    reg64_xmm2,
    reg64_xmm3,
    reg64_xmm4,
    reg64_xmm5,
    reg64_xmm6,
    reg64_xmm7,
    reg64_xmm8,
    reg64_xmm9,
    reg64_xmm10,
    reg64_xmm11,
    reg64_xmm12,
    reg64_xmm13,
    reg64_xmm14,
    reg64_xmm15,

    reg64_tmm0,
    reg64_tmmmax = 31,
} x64vr;

constexpr u8 to_ureg(const u8 reg)                 { return (reg + 1); }
constexpr u8 to_ureg(const u8 reg, const u8 valid) { return (valid ? (reg + 1) : 0); }

// -> lut?
constexpr u8 to_core_except(const u8 x64_ex)
{
    (void) x64_ex;
    return ex_UNSPEC;
};

typedef enum
{
    pd_reset,  // ready
    pd_prefix, // parse prefixes
    pd_opcode, // opcode bytes
    pd_modrm,  // modrm, sib, displacement
    pd_imm,    // imm bytes
} pd_state;

// TODO msrom latency
struct x64Decoder
{
    x64op    instr = zero_x64op; // instruction to decode
    const u8 type;               // x64d_type
    u8       busy  = 0;
    const u8 id;                 // unique id

    x64Decoder(u8 id, u8 type) : type(type), id(id) {};
}; // x64Decoder

const std::string x64d_type_str[3] =
{
    ("fast"), ("complex"), ("MSROM")
};


// need:
// - predecode
// = iqueue
// - fuse
// - decode
// = uop buffer
// - fuse
// -> uqueue

struct DecoderStation
{
    vector<x64Decoder> decoders =
    {
        x64Decoder(0, x64d_fast),
        x64Decoder(1, x64d_fast),
        x64Decoder(2, x64d_fast),
        x64Decoder(3, x64d_fast),
        x64Decoder(4, x64d_cmplx),
        x64Decoder(5, x64d_seq),
    };
};

std::ostream& operator<<(std::ostream& os, const DecoderStation& ds);

class x64Frontend : public Frontend
{
    public:
    x64Frontend(MemoryManager& mmu, LatchQueue<uop>* uqueue,
            Simulator::SimulatorState& state);
    ~x64Frontend();
    u8                cycle();
    u8                flush();
    std::stringstream summary();

    u8 fetch();
    u8 fuse_macro();
    u8 udecode();
    u8 fuse_micro();

    u8 get_tmpreg(const u8 regcls);
    u8 run_decode(const x64op& op);

    private:
    u8                  flush(u8 total);

    LatchQueue<x64op>   iqueue = LatchQueue<x64op>(IQUEUE_SIZE);
    DecoderStation      ds;

    vector<u8>          fetchbytes;   // fetch queue aka pd buffer
    u64                 pdblocksz;    // current block size
    u8                  pd_state;     // last predecoder state
    u8                  pd_remaining; // remaining displ/imm bytes
    x64op               part_op;      // last partially decoded instruction
    std::deque<u8>      next_decoder; // decoder which places uops into the uqueue next
    u8                  msrip;        // next uop index for current macro op

    // one per regfile
    u8                  cur_tmp_gp;   // last used temporary gpreg
    u8                  cur_tmp_vr;
    // u8                  cur_tmp_fp;
};

#endif // SIM_FRONTEND64_H
