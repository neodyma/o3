// o3 RISC simulator
// 
// x64 frontend
// - opcode tables
//
// Lukas Heine 2021

#ifndef SIM_TABLES64_H
#define SIM_TABLES64_H

#include "../types.hh"

namespace x64def
{
    // addressing modes
    typedef enum
    {
        A = 1,  // direct, no modrm
        B,      // vex.vvvv  -> gp
        C,      // modrm.reg -> control
        D,      // modrm.reg -> debug
        E,      // modrm.rm  -> gp / mem
        F,      // flags
        G,      // modrm.reg -> gp register
        H,      // vex.vvvv  -> xmm/ymm (no legacy)
        I,      // immediate
        J,      // rip relative
        L,      // imm[7:4]  -> xmm/ymm
        M,      // modrm.rm  -> mem only
        N,      // modrm.rm  -> MMX pq
        O,      // no modrm
        P,      // modrm.reg -> MMX pq
        Q,      // modrm.rm  -> mm / mem
        R,      // modrm.rm  -> gp only
        S,      // modrm.reg -> segment
        U,      // modrm.rm  -> xmm/ymm
        V,      // modrm.reg -> xmm/ymm
        W,      // modrm.rm  -> xymm / mem
        X,      // ds:si
        Y,      // es:di

        Z,      // fixed register operand
    } addr_modes;

    typedef enum
    {   // immediates: >127 -> determined by opsize
        b  =   1, // 1      byte
        c  = 128, // 1/2    byte/word, never used
        d  =   4, // 4      dword
        q  =   8, // 8      qword
        v  = 129, // 2/4/8  word/dword/qword
        w  =   2, // 2      word
        z  = 130, // 2/4/4  w16/d32/d64

        e  =   3, // 3      w + b, only used by ENTER (c8)
        f  = 131, // 2/8    word/qword (e.g. 8f pop)
        g  = 132, // 1/2/4  byte/word/dword (e.g. cbw)

        // vector operand types
        dq =  16, // 16     double qword
        pd = 133, // f64p   packed double
        pi =   8, // i32p   qword packed int
        ps = 134, // f32p   packed single
        qq =  32, // 32     quad qword
        sd =   8, // 8      scalar double
        ss =   4, // 4      scalar single
        si =   4, // 4      scalar int
    } op_types;

#define _ 0

    // sdm 2d - a.3

    // one byte opcode group identifiers
    constexpr u8 opgrp_1b(u8 byte)
    {
        switch(byte)
        {
            default:
                return 0;
            case 0x80: case 0x81: case 0x82: case 0x83:
                return 1; // immediate group 1
            case 0x8f:
                return 1; // group 1a, only pop Ev for now
            case 0xc0: case 0xc1:
            case 0xd0: case 0xd1: case 0xd2: case 0xd3:
                return 2; // shift group 2
            case 0xf6: case 0xf7:
                return 3; // unary group 3
            case 0xfe:
                return 4; // inc/dec group 4
            case 0xff:
                return 5; // inc/dec group 5
            case 0xc6: case 0xc7:
                return 11; // mov group 11
        }
    }

    // two byte opcode group identifiers
    constexpr u8 opgrp_2b(u8 byte)
    {
        switch(byte)
        {
            default:   return  0;
            case 0x00: return  6;
            case 0x01: return  7;
            case 0xba: return  8;
            case 0xc7: return  9;
            case 0xb9: return 10;
            case 0x71: return 12;
            case 0x72: return 13;
            case 0x73: return 14;
            case 0xae: return 15;
            case 0x18: return 16;
        }
    }

    // one byte opcodes, no escape
    // table a-2
    constexpr u8 modrm_1b[256] =
    {   // modrm: C, (D), E, G, M, N, P, Q, R, S, U, V, W
        1,1,1,1,_,_,_,_,1,1,1,1,_,_,_,_,1,1,1,1,_,_,_,_,1,1,1,1,_,_,_,_, // 00 - 1f
        1,1,1,1,_,_,_,_,1,1,1,1,_,_,_,_,1,1,1,1,_,_,_,_,1,1,1,1,_,_,_,_, // 20 - 3f
        _,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_, // 40 - 5f
        _,_,1,1,_,_,_,_,_,1,_,1,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_, // 60 - 7f
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_, // 80 - 9f
        _,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_, // a0 - bf
        1,1,_,_,1,1,1,1,_,_,_,_,_,_,_,_,1,1,1,1,_,_,_,_,1,1,1,1,1,1,1,1, // c0 - df
        _,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,1,1,_,_,_,_,_,_,1,1  // e0 - ff
    };

    // two byte opcodes, escape 0f
    // table a-3
    constexpr u8 modrm_2b[256] =
    {
        1,1,1,1,_,_,_,_,_,_,_,_,_,1,_,_,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, // 00 - 1f
        1,1,1,1,_,_,_,_,1,1,1,1,1,1,1,1,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_, // 20 - 3f
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, // 40 - 5f
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,_,1,1,_,_,1,1,1,1, // 60 - 7f
        _,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, // 80 - 9f
        _,_,_,1,1,1,_,_,_,_,_,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, // a0 - bf
        1,1,1,1,1,1,1,1,_,_,_,_,_,_,_,_,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, // c0 - df
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,_  // e0 - ff
    };

    // three bytes (0f 38 _ or 0f 3a _) always use modrm if defined
    // tables are not needed for <= SSE2 and are significantly more complex
    // since some instructions are only valid with prefix

    // TODO group lookups

    // TODO modrm addressing table

    // TODO sib addressing table

    // one byte opcodes
    constexpr u8 immsz_1b[256] =
    {   // immediates: I (b, c, d, q, v, w, z), Lx (one byte), O
        _,_,_,_,b,z,_,_,_,_,_,_,b,z,_,_,_,_,_,_,b,z,_,_,_,_,_,_,b,z,_,_, // 00 - 1f
        _,_,_,_,b,z,_,_,_,_,_,_,b,z,_,_,_,_,_,_,b,z,_,_,_,_,_,_,b,z,_,_, // 20 - 3f
        _,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_, // 40 - 5f
        _,_,_,_,_,_,_,_,z,z,b,b,_,_,_,_,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b, // 60 - 7f
        b,z,_,b,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_, // 80 - 9f
        q,q,q,q,_,_,_,_,b,z,_,_,_,_,_,_,b,b,b,b,b,b,b,b,v,v,v,v,v,v,v,v, // a0 - bf
        b,b,w,_,_,_,b,z,e,_,w,_,_,b,_,_,_,_,_,_,b,b,_,_,_,_,_,_,_,_,_,_, // c0 - df
        _,_,_,b,b,b,b,b,z,z,_,b,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_  // e0 - ff
    };

    // two byte opcodes
    // immediates can only be one byte in size here, except for jcc
    // todo setcc, maybe just do a table for this..
    constexpr u8 immsz_2b(u8 byte)
    {
        if((byte >= 0x80 && byte < 0x90 ))
            return z;

        return ((byte >= 0x70 && byte < 0x74) ||
                (byte == 0xba) ||
                (byte == 0xc2) ||
                (byte >= 0xc4 && byte < 0xc7));
    }

    // three byte opcode, second escape is 38
    // three byte opcode, second escape is 3a
    // see three-byte modrm

    // mandatory prefix used
    constexpr u8 reqpfx_1b(u8 byte) { (void) byte; return 0; };

    constexpr u8 reqpfx_2b[256] =
    {
        _,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,1,1,1,1,1,1,1,1,1,_,1,1,_,_,_,_, // 00 - 1f
        _,_,_,_,_,_,_,_,1,1,1,1,1,1,1,1,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_, // 20 - 3f
        _,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, // 40 - 5f
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,_,_,_,_,_,1,1,1,1, // 60 - 7f
        _,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_, // 80 - 9f
        _,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,1,_,_,_,1,1,_,_, // a0 - bf
        _,_,1,_,1,1,1,1,_,_,_,_,_,_,_,_,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, // c0 - df
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0  // e0 - ff
    };


#undef _

struct x64operand
{
    u8 addr_mode    = 0;
    u8 operand_type = 0;

    operator bool() const
    {
        return this->addr_mode && this->operand_type;
    }

    auto operator==(const x64operand& op) const
    {
        return this->addr_mode == op.addr_mode &&
            this->operand_type == op.operand_type;
    }
};

struct x64opinfo
{
    string             mnemonic;
    vector<x64operand> operands = {};

    operator bool() const
    {
        return !this->mnemonic.empty();
    }

    auto operator==(const x64opinfo& info) const
    {
        return this->mnemonic == info.mnemonic &&
            this->operands == info.operands;
    }
};

const x64opinfo zero_x64opinfo = { "", {} };

constexpr u8 is_rmop(x64operand op)
{
    return (op.addr_mode == E || op.addr_mode == M || op.addr_mode == Q || op.addr_mode == W);
}

constexpr u8 is_immop(x64operand op)    { return (op.addr_mode == I) || (op.addr_mode == J); }

// get modrm.rm operand with possible memory access and its position
inline pair<x64operand, u8> get_rmop(vector<x64operand> operands)
{
    u8 i = 0;
    for(auto op : operands)
    {
        if(is_rmop(op))
            return std::make_pair(op, i);
        i++;
    }

    return std::make_pair((x64operand)0, 0);
}

// map required prefixes, escape, opcode bytes (plus relevant modrm bits) -> opinfo
// simple lookup: { (prefixes), opcodes }
// group lookup:  { (prefixes), opcodes, modrm bits }
// const std::pair<vector<u8>, x64opinfo> x64opmap[768] =
// instrs marked with line comment are not implemented yet
const std::map<vector<u8>, x64opinfo> x64opmap =
{
    // one byte 
    { {0x00},               { "add",                {{E,b}, {G,b}}          }},
    { {0x01},               { "add",                {{E,v}, {G,v}}          }},
    { {0x02},               { "add",                {{G,b}, {E,b}}          }},
    { {0x03},               { "add",                {{G,v}, {E,v}}          }},
    { {0x04},               { "add",                {{Z,b}, {I,b}}          }},
    { {0x05},               { "add",                {{Z,v}, {I,z}}          }},
    { {0x08},               { "or",                 {{E,b}, {G,b}}          }},
    { {0x09},               { "or",                 {{E,v}, {G,v}}          }},
    { {0x0a},               { "or",                 {{G,b}, {E,b}}          }},
    { {0x0b},               { "or",                 {{G,v}, {E,v}}          }},
    { {0x0c},               { "or",                 {{Z,b}, {I,b}}          }},
    { {0x0d},               { "or",                 {{Z,v}, {I,z}}          }},
    // 0x0f                    2-byte escape
    
    { {0x10},               { "adc",                {{E,b}, {G,b}}          }},
    { {0x11},               { "adc",                {{E,v}, {G,v}}          }},
    { {0x12},               { "adc",                {{G,b}, {E,b}}          }},
    { {0x13},               { "adc",                {{G,v}, {E,v}}          }},
    { {0x14},               { "adc",                {{Z,b}, {I,b}}          }},
    { {0x15},               { "adc",                {{Z,v}, {I,z}}          }},
    { {0x18},               { "sbb",                {{E,b}, {G,b}}          }},
    { {0x19},               { "sbb",                {{E,v}, {G,v}}          }},
    { {0x1a},               { "sbb",                {{G,b}, {E,b}}          }},
    { {0x1b},               { "sbb",                {{G,v}, {E,v}}          }},
    { {0x1c},               { "sbb",                {{Z,b}, {I,b}}          }},
    { {0x1d},               { "sbb",                {{Z,v}, {I,z}}          }},

    { {0x20},               { "and",                {{E,b}, {G,b}}          }},
    { {0x21},               { "and",                {{E,v}, {G,v}}          }},
    { {0x22},               { "and",                {{G,b}, {E,b}}          }},
    { {0x23},               { "and",                {{G,v}, {E,v}}          }},
    { {0x24},               { "and",                {{Z,b}, {I,b}}          }},
    { {0x25},               { "and",                {{Z,v}, {I,z}}          }},
    { {0x28},               { "sub",                {{E,b}, {G,b}}          }},
    { {0x29},               { "sub",                {{E,v}, {G,v}}          }},
    { {0x2a},               { "sub",                {{G,b}, {E,b}}          }},
    { {0x2b},               { "sub",                {{G,v}, {E,v}}          }},
    { {0x2c},               { "sub",                {{Z,b}, {I,b}}          }},
    { {0x2d},               { "sub",                {{Z,v}, {I,z}}          }},

    { {0x30},               { "xor",                {{E,b}, {G,b}}          }},
    { {0x31},               { "xor",                {{E,v}, {G,v}}          }},
    { {0x32},               { "xor",                {{G,b}, {E,b}}          }},
    { {0x33},               { "xor",                {{G,v}, {E,v}}          }},
    { {0x34},               { "xor",                {{Z,b}, {I,b}}          }},
    { {0x35},               { "xor",                {{Z,v}, {I,z}}          }},
    { {0x38},               { "cmp",                {{E,b}, {G,b}}          }},
    { {0x39},               { "cmp",                {{E,v}, {G,v}}          }},
    { {0x3a},               { "cmp",                {{G,b}, {E,b}}          }},
    { {0x3b},               { "cmp",                {{G,v}, {E,v}}          }},
    { {0x3c},               { "cmp",                {{Z,b}, {I,b}}          }},
    { {0x3d},               { "cmp",                {{Z,v}, {I,z}}          }},

    // 0x4_                    REX

    { {0x50},               { "push",               {{Z,f}}                 }},
    { {0x51},               { "push",               {{Z,f}}                 }},
    { {0x52},               { "push",               {{Z,f}}                 }},
    { {0x53},               { "push",               {{Z,f}}                 }},
    { {0x54},               { "push",               {{Z,f}}                 }},
    { {0x55},               { "push",               {{Z,f}}                 }},
    { {0x56},               { "push",               {{Z,f}}                 }},
    { {0x57},               { "push",               {{Z,f}}                 }},
    { {0x58},               { "pop",                {{Z,f}}                 }},
    { {0x59},               { "pop",                {{Z,f}}                 }},
    { {0x5a},               { "pop",                {{Z,f}}                 }},
    { {0x5b},               { "pop",                {{Z,f}}                 }},
    { {0x5c},               { "pop",                {{Z,f}}                 }},
    { {0x5d},               { "pop",                {{Z,f}}                 }},
    { {0x5e},               { "pop",                {{Z,f}}                 }},
    { {0x5f},               { "pop",                {{Z,f}}                 }},

    { {0x63},               { "movsxd",             {{G,v}, {E,z}}          }}, // not Ev
    { {0x68},               { "push",               {{I,f}}                 }}, // NOT IZ
    { {0x69},               { "imul",               {{G,v}, {E,v}, {I,z}}   }},
    { {0x6a},               { "push",               {{I,b}}                 }},
    { {0x6b},               { "imul",               {{G,v}, {E,v}, {I,b}}   }},
    // 0x6c..                  ins/outs

    // short jumps
    { {0x70},               { "jo",                 {{J,b}}                 }},
    { {0x71},               { "jno",                {{J,b}}                 }},
    { {0x72},               { "jb",                 {{J,b}}                 }},
    { {0x73},               { "jnb",                {{J,b}}                 }},
    { {0x74},               { "jz",                 {{J,b}}                 }},
    { {0x75},               { "jnz",                {{J,b}}                 }},
    { {0x76},               { "jbe",                {{J,b}}                 }},
    { {0x77},               { "jnbe",               {{J,b}}                 }},
    { {0x78},               { "js",                 {{J,b}}                 }},
    { {0x79},               { "jns",                {{J,b}}                 }},
    { {0x7a},               { "jp",                 {{J,b}}                 }},
    { {0x7b},               { "jnp",                {{J,b}}                 }},
    { {0x7c},               { "jl",                 {{J,b}}                 }},
    { {0x7d},               { "jnl",                {{J,b}}                 }},
    { {0x7e},               { "jle",                {{J,b}}                 }},
    { {0x7f},               { "jnle",               {{J,b}}                 }},

    // immediate group 1
    { {0x80, 0b000},        { "add",                {{E,b}, {I,b}}          }},
    { {0x80, 0b001},        { "or",                 {{E,b}, {I,b}}          }},
    { {0x80, 0b010},        { "adc",                {{E,b}, {I,b}}          }},
    { {0x80, 0b011},        { "sbb",                {{E,b}, {I,b}}          }},
    { {0x80, 0b100},        { "and",                {{E,b}, {I,b}}          }},
    { {0x80, 0b101},        { "sub",                {{E,b}, {I,b}}          }},
    { {0x80, 0b110},        { "xor",                {{E,b}, {I,b}}          }},
    { {0x80, 0b111},        { "cmp",                {{E,b}, {I,b}}          }},
    { {0x81, 0b000},        { "add",                {{E,v}, {I,z}}          }},
    { {0x81, 0b001},        { "or",                 {{E,v}, {I,z}}          }},
    { {0x81, 0b010},        { "adc",                {{E,v}, {I,z}}          }},
    { {0x81, 0b011},        { "sbb",                {{E,v}, {I,z}}          }},
    { {0x81, 0b100},        { "and",                {{E,v}, {I,z}}          }},
    { {0x81, 0b101},        { "sub",                {{E,v}, {I,z}}          }},
    { {0x81, 0b110},        { "xor",                {{E,v}, {I,z}}          }},
    { {0x81, 0b111},        { "cmp",                {{E,v}, {I,z}}          }},
    { {0x83, 0b000},        { "add",                {{E,v}, {I,b}}          }},
    { {0x83, 0b001},        { "or",                 {{E,v}, {I,b}}          }},
    { {0x83, 0b010},        { "adc",                {{E,v}, {I,b}}          }},
    { {0x83, 0b011},        { "sbb",                {{E,v}, {I,b}}          }},
    { {0x83, 0b100},        { "and",                {{E,v}, {I,b}}          }},
    { {0x83, 0b101},        { "sub",                {{E,v}, {I,b}}          }},
    { {0x83, 0b110},        { "xor",                {{E,v}, {I,b}}          }},
    { {0x83, 0b111},        { "cmp",                {{E,v}, {I,b}}          }},
    { {0x84},               { "test",               {{E,b}, {G,b}}          }},
    { {0x85},               { "test",               {{E,v}, {G,v}}          }},
    { {0x86},               { "xchg",               {{E,b}, {G,b}}          }},
    { {0x87},               { "xchg",               {{E,v}, {G,v}}          }},
    { {0x88},               { "mov",                {{E,b}, {G,b}}          }},
    { {0x89},               { "mov",                {{E,v}, {G,v}}          }},
    { {0x8a},               { "mov",                {{G,b}, {E,b}}          }},
    { {0x8b},               { "mov",                {{G,v}, {E,v}}          }},
    { {0x8c},               { "mov",                {{E,v}, {S,w}}          }},
    { {0x8d},               { "lea",                {{G,v}, {M,v}}          }},
    { {0x8e},               { "mov",                {{S,w}, {E,w}}          }},
    // group 1a
    { {0x8f, 0b000},        { "pop",                {{E,f}}                 }}, // not Ev

    // xchg 9_: rx <-> _AX depending on REX and 66
    { {0x90},               { "xchg",               {{R,v}, {R,v}}          }},
    { {0x91},               { "xchg",               {{R,v}, {R,v}}          }},
    { {0x92},               { "xchg",               {{R,v}, {R,v}}          }},
    { {0x93},               { "xchg",               {{R,v}, {R,v}}          }},
    { {0x94},               { "xchg",               {{R,v}, {R,v}}          }},
    { {0x95},               { "xchg",               {{R,v}, {R,v}}          }},
    { {0x96},               { "xchg",               {{R,v}, {R,v}}          }},
    { {0x97},               { "xchg",               {{R,v}, {R,v}}          }},
    { {0x98},               { "cbw",                {{Z,g}}                 }}, //
    { {0x99},               { "cwd",                {{Z,v}}                 }}, //
    { {0x9c},               { "pushf",              {{F,f}}                 }}, //
    { {0x9d},               { "popf",               {{F,f}}                 }}, //
    // 0x9e                    sahf/lahf

    // moffset moves, displ64
    { {0xa0},               { "mov",                {{Z,b}, {O,b}}          }},
    { {0xa1},               { "mov",                {{Z,v}, {O,v}}          }},
    { {0xa2},               { "mov",                {{O,b}, {Z,b}}          }},
    { {0xa3},               { "mov",                {{O,v}, {Z,v}}          }},
    // 0xa4..                  movs, cmps
    { {0xa8},               { "test",               {{Z,b}, {I,b}}          }},
    { {0xa9},               { "test",               {{Z,v}, {I,z}}          }},
    // 0xaa..                  stos, lods, scas

    // immediate moves, registers encoded
    { {0xb0},               { "mov",                {{Z,b}, {I,b}}          }},
    { {0xb1},               { "mov",                {{Z,b}, {I,b}}          }},
    { {0xb2},               { "mov",                {{Z,b}, {I,b}}          }},
    { {0xb3},               { "mov",                {{Z,b}, {I,b}}          }},
    { {0xb4},               { "mov",                {{Z,b}, {I,b}}          }},
    { {0xb5},               { "mov",                {{Z,b}, {I,b}}          }},
    { {0xb6},               { "mov",                {{Z,b}, {I,b}}          }},
    { {0xb7},               { "mov",                {{Z,b}, {I,b}}          }},
    { {0xb8},               { "mov",                {{Z,v}, {I,v}}          }},
    { {0xb9},               { "mov",                {{Z,v}, {I,v}}          }},
    { {0xba},               { "mov",                {{Z,v}, {I,v}}          }},
    { {0xbb},               { "mov",                {{Z,v}, {I,v}}          }},
    { {0xbc},               { "mov",                {{Z,v}, {I,v}}          }},
    { {0xbd},               { "mov",                {{Z,v}, {I,v}}          }},
    { {0xbe},               { "mov",                {{Z,v}, {I,v}}          }},
    { {0xbf},               { "mov",                {{Z,v}, {I,v}}          }},

    // shift group 2
    { {0xc0, 0b000},        { "rol",                {{E,b}, {I,b}}          }},
    { {0xc0, 0b001},        { "ror",                {{E,b}, {I,b}}          }},
    { {0xc0, 0b010},        { "rcl",                {{E,b}, {I,b}}          }},
    { {0xc0, 0b011},        { "rcr",                {{E,b}, {I,b}}          }},
    { {0xc0, 0b100},        { "shl",                {{E,b}, {I,b}}          }},
    { {0xc0, 0b101},        { "shr",                {{E,b}, {I,b}}          }},
    { {0xc0, 0b111},        { "sar",                {{E,b}, {I,b}}          }},
    { {0xc1, 0b000},        { "rol",                {{E,v}, {I,b}}          }},
    { {0xc1, 0b001},        { "ror",                {{E,v}, {I,b}}          }},
    { {0xc1, 0b010},        { "rcl",                {{E,v}, {I,b}}          }},
    { {0xc1, 0b011},        { "rcr",                {{E,v}, {I,b}}          }},
    { {0xc1, 0b100},        { "shl",                {{E,v}, {I,b}}          }},
    { {0xc1, 0b101},        { "shr",                {{E,v}, {I,b}}          }},
    { {0xc1, 0b111},        { "sar",                {{E,v}, {I,b}}          }},

    { {0xc2},               { "ret",                {{I,w}}                 }},
    { {0xc3},               { "ret"                                         }},
    // 0xc4/c5                 VEX
    // mov group 11
    { {0xc6, 0b000},        { "mov",                {{E,b}, {I,b}}          }}, // xabort idk
    { {0xc7, 0b000},        { "mov",                {{E,v}, {I,z}}          }},
    { {0xc8},               { "enter",              {{I,w}, {I,b}}          }}, //
    { {0xc9},               { "leave"                                       }}, //
    { {0xca},               { "ret",                {{I,w}}                 }}, //
    { {0xcb},               { "ret"                                         }}, //
    { {0xcc},               { "int3"                                        }},
    { {0xcd},               { "int",                {{I,b}}                 }},
    { {0xcf},               { "iret"                                        }}, //

    // shift group 2 again
    { {0xd0, 0b000},        { "rol",                {{E,b}}                 }},
    { {0xd0, 0b001},        { "ror",                {{E,b}}                 }},
    { {0xd0, 0b010},        { "rcl",                {{E,b}}                 }},
    { {0xd0, 0b011},        { "rcr",                {{E,b}}                 }},
    { {0xd0, 0b100},        { "shl",                {{E,b}}                 }},
    { {0xd0, 0b101},        { "shr",                {{E,b}}                 }},
    { {0xd0, 0b111},        { "sar",                {{E,b}}                 }},
    { {0xd1, 0b000},        { "rol",                {{E,v}}                 }},
    { {0xd1, 0b001},        { "ror",                {{E,v}}                 }},
    { {0xd1, 0b010},        { "rcl",                {{E,v}}                 }},
    { {0xd1, 0b011},        { "rcr",                {{E,v}}                 }},
    { {0xd1, 0b100},        { "shl",                {{E,v}}                 }},
    { {0xd1, 0b101},        { "shr",                {{E,v}}                 }},
    { {0xd1, 0b111},        { "sar",                {{E,v}}                 }},
    { {0xd2, 0b000},        { "rol",                {{E,b}, {Z,b}}          }},
    { {0xd2, 0b001},        { "ror",                {{E,b}, {Z,b}}          }},
    { {0xd2, 0b010},        { "rcl",                {{E,b}, {Z,b}}          }},
    { {0xd2, 0b011},        { "rcr",                {{E,b}, {Z,b}}          }},
    { {0xd2, 0b100},        { "shl",                {{E,b}, {Z,b}}          }},
    { {0xd2, 0b101},        { "shr",                {{E,b}, {Z,b}}          }},
    { {0xd2, 0b111},        { "sar",                {{E,b}, {Z,b}}          }},
    { {0xd3, 0b000},        { "rol",                {{E,v}, {Z,b}}          }},
    { {0xd3, 0b001},        { "ror",                {{E,v}, {Z,b}}          }},
    { {0xd3, 0b010},        { "rcl",                {{E,v}, {Z,b}}          }},
    { {0xd3, 0b011},        { "rcr",                {{E,v}, {Z,b}}          }},
    { {0xd3, 0b100},        { "shl",                {{E,v}, {Z,b}}          }},
    { {0xd3, 0b101},        { "shr",                {{E,v}, {Z,b}}          }},
    { {0xd3, 0b111},        { "sar",                {{E,v}, {Z,b}}          }},
    // 0xd7                    xlat 
    // 0xd8..                  ESC

    // { {0xe0},               { "loopne",             {{J,b}, {R,v}}          }},
    // { {0xe1},               { "loope",              {{J,b}, {R,v}}          }},
    // { {0xe2},               { "loop",               {{J,b}, {R,v}}          }},
    { {0xe3},               { "jrcxz",              {{J,b}, {R,v}}          }},
    // 0xe4..                  in/out
    { {0xe8},               { "call",               {{J,z}}                 }},
    { {0xe9},               { "jmp",                {{J,z}}                 }},
    { {0xeb},               { "jmp",                {{J,b}}                 }},
    // 0xec..                  in/out

    { {0xf1},               { "int1"                                        }},
    { {0xf4},               { "halt"                                        }},
    { {0xf5},               { "cmc"                                         }},
    // unary group 3
    { {0xf6, 0b000},        { "test",               {{E,b}, {I,b}}          }},
    { {0xf6, 0b010},        { "not",                {{E,b}}                 }},
    { {0xf6, 0b011},        { "neg",                {{E,b}}                 }},
    { {0xf6, 0b100},        { "mul",                {{E,b}}                 }},
    { {0xf6, 0b101},        { "imul",               {{E,b}}                 }},
    { {0xf6, 0b110},        { "div",                {{E,b}}                 }},
    { {0xf6, 0b111},        { "idiv",               {{E,b}}                 }},
    { {0xf7, 0b000},        { "test",               {{E,v}, {I,z}}          }},
    { {0xf7, 0b010},        { "not",                {{E,v}}                 }},
    { {0xf7, 0b011},        { "neg",                {{E,v}}                 }},
    { {0xf7, 0b100},        { "mul",                {{E,v}}                 }},
    { {0xf7, 0b101},        { "imul",               {{E,v}}                 }},
    { {0xf7, 0b110},        { "div",                {{E,v}}                 }},
    { {0xf7, 0b111},        { "idiv",               {{E,v}}                 }},
    { {0xf8},               { "clc",                {{F,q}}                 }},
    { {0xf9},               { "stc",                {{F,q}}                 }},
    { {0xfa},               { "cli",                {{F,q}}                 }},
    { {0xfb},               { "sti",                {{F,q}}                 }},
    { {0xfc},               { "cld",                {{F,q}}                 }},
    { {0xfd},               { "std",                {{F,q}}                 }},

    // inc/dec group 4
    { {0xfe, 0b000},        { "inc",                {{E,b}}                 }},
    { {0xfe, 0b001},        { "dec",                {{E,b}}                 }},
    // inc/dec group 5
    { {0xff, 0b000},        { "inc",                {{E,v}}                 }},
    { {0xff, 0b001},        { "dec",                {{E,v}}                 }},
    { {0xff, 0b010},        { "call",               {{E,v}}                 }},
    { {0xff, 0b011},        { "call",               {{E,v}}                 }}, //
    { {0xff, 0b100},        { "jmp",                {{E,v}}                 }},
    { {0xff, 0b101},        { "jmp",                {{M,v}}                 }}, //


    // two byte opcodes
    // segment control group 6
    // segment/vm control group 7
    // 0x0f, 0x02..            segment access control
    // { {0x0f, 0x05},         { "syscall"                                     }}, //
    // 0x0f, 0x06              task switch flag
    // { {0x0f, 0x07},         { "sysret"                                      }}, //
    { {0x0f, 0x08},         { "invd"                                        }}, //
    { {0x0f, 0x09},         { "wbinvd"                                      }}, //
    { {0x0f, 0x0a},         { "ud2"                                         }},
    { {0x0f, 0x0d},         { "prefetchw",          {{E,v}}                 }}, //
    
    // 0x0f, 0x10..            TODO
    // 0x0f, 0x18              prefetch group 16
    { {0x0f, 0x19},         { "nop reserved"                                }},
    // 0xf0, 0x1a..            MPX
    { {0x0f, 0x1c},         { "nop reserved"                                }},
    { {0x0f, 0x1d},         { "nop reserved"                                }},
    { {0x0f, 0x1e},         { "nop reserved"                                }},
    { {0x0f, 0x1f},         { "nop /0",             {{E,v}}                 }},
    
    // 0x0f, 0x20..            control mov
    { {0x0f, 0x31},         { "rdtsc"                                       }},

    { {0x0f, 0x40},         { "cmovo",              {{G,v},{E,v}}           }},
    { {0x0f, 0x41},         { "cmovno",             {{G,v},{E,v}}           }},
    { {0x0f, 0x42},         { "cmovb",              {{G,v},{E,v}}           }},
    { {0x0f, 0x43},         { "cmovnb",             {{G,v},{E,v}}           }},
    { {0x0f, 0x44},         { "cmovz",              {{G,v},{E,v}}           }},
    { {0x0f, 0x45},         { "cmovnz",             {{G,v},{E,v}}           }},
    { {0x0f, 0x46},         { "cmovbe",             {{G,v},{E,v}}           }},
    { {0x0f, 0x47},         { "cmovnbe",            {{G,v},{E,v}}           }},
    { {0x0f, 0x48},         { "cmovs",              {{G,v},{E,v}}           }},
    { {0x0f, 0x49},         { "cmovns",             {{G,v},{E,v}}           }},
    { {0x0f, 0x4a},         { "cmovp",              {{G,v},{E,v}}           }},
    { {0x0f, 0x4b},         { "cmovnp",             {{G,v},{E,v}}           }},
    { {0x0f, 0x4c},         { "cmovl",              {{G,v},{E,v}}           }},
    { {0x0f, 0x4d},         { "cmovnl",             {{G,v},{E,v}}           }},
    { {0x0f, 0x4e},         { "cmovle",             {{G,v},{E,v}}           }},
    { {0x0f, 0x4f},         { "cmovnle",            {{G,v},{E,v}}           }},

    // long jumps
    { {0x0f, 0x80},         { "jo",                 {{J,d}}                 }},
    { {0x0f, 0x81},         { "jno",                {{J,d}}                 }},
    { {0x0f, 0x82},         { "jb",                 {{J,d}}                 }},
    { {0x0f, 0x83},         { "jnb",                {{J,d}}                 }},
    { {0x0f, 0x84},         { "jz",                 {{J,d}}                 }},
    { {0x0f, 0x85},         { "jnz",                {{J,d}}                 }},
    { {0x0f, 0x86},         { "jbe",                {{J,d}}                 }},
    { {0x0f, 0x87},         { "jnbe",               {{J,d}}                 }},
    { {0x0f, 0x88},         { "js",                 {{J,d}}                 }},
    { {0x0f, 0x89},         { "jns",                {{J,d}}                 }},
    { {0x0f, 0x8a},         { "jp",                 {{J,d}}                 }},
    { {0x0f, 0x8b},         { "jnp",                {{J,d}}                 }},
    { {0x0f, 0x8c},         { "jl",                 {{J,d}}                 }},
    { {0x0f, 0x8d},         { "jnl",                {{J,d}}                 }},
    { {0x0f, 0x8e},         { "jle",                {{J,d}}                 }},
    { {0x0f, 0x8f},         { "jnle",               {{J,d}}                 }},

    { {0x0f, 0x90},         { "seto",               {{E,b}}                 }}, //
    { {0x0f, 0x91},         { "setno",              {{E,b}}                 }}, //
    { {0x0f, 0x92},         { "setb",               {{E,b}}                 }}, //
    { {0x0f, 0x93},         { "setnb",              {{E,b}}                 }}, //
    { {0x0f, 0x94},         { "setz",               {{E,b}}                 }}, //
    { {0x0f, 0x95},         { "setnz",              {{E,b}}                 }}, //
    { {0x0f, 0x96},         { "setbe",              {{E,b}}                 }}, //
    { {0x0f, 0x97},         { "setnbe",             {{E,b}}                 }}, //
    { {0x0f, 0x98},         { "sets",               {{E,b}}                 }}, //
    { {0x0f, 0x99},         { "setns",              {{E,b}}                 }}, //
    { {0x0f, 0x9a},         { "setp",               {{E,b}}                 }}, //
    { {0x0f, 0x9b},         { "setnp",              {{E,b}}                 }}, //
    { {0x0f, 0x9c},         { "setl",               {{E,b}}                 }}, //
    { {0x0f, 0x9d},         { "setnl",              {{E,b}}                 }}, //
    { {0x0f, 0x9e},         { "setle",              {{E,b}}                 }}, //
    { {0x0f, 0x9f},         { "setnle",             {{E,b}}                 }}, //

    { {0x0f, 0xa0},         { "pushfs",             {{Z,f}}                 }}, //
    { {0x0f, 0xa1},         { "popfs",              {{Z,f}}                 }}, //
    { {0x0f, 0xa2},         { "cpuid"                                       }}, //
    { {0x0f, 0xa3},         { "bt",                 {{E,v},{G,v}}           }}, //
    { {0x0f, 0xa4},         { "shld",               {{E,v},{G,v},{I,b}}     }}, //
    { {0x0f, 0xa5},         { "shld",               {{E,v},{G,v},{Z,b}}     }}, //
    { {0x0f, 0xa8},         { "pushgs",             {{Z,f}}                 }}, //
    { {0x0f, 0xa9},         { "popgs",              {{Z,f}}                 }}, //
    { {0x0f, 0xab},         { "bts",                {{E,v},{G,v}}           }}, //
    { {0x0f, 0xac},         { "shrd",               {{E,v},{G,v},{I,b}}     }}, //
    { {0x0f, 0xad},         { "shrd",               {{E,v},{G,v},{Z,b}}     }}, //
    { {0x0f, 0xaf},         { "imul",               {{G,v},{E,v}}           }}, //

    { {0x0f, 0xb0},         { "cmpxchg",            {{E,b},{G,b}}           }}, //
    { {0x0f, 0xb1},         { "cmpxchg",            {{E,v},{G,v}}           }}, //
    // lss ..
    { {0x0f, 0xb3},         { "btr",                {{E,v},{G,v}}           }}, //
    // lfs
    // lgs
    { {0x0f, 0xb6},         { "movzx",              {{G,v},{E,b}}           }}, //
    { {0x0f, 0xb7},         { "movzx",              {{G,v},{E,w}}           }}, //
    { {0xf3, 0x0f, 0xb8},   { "popcnt",             {{G,v},{E,v}}           }}, //
    { {0x0f, 0xb9},         { "ud1"                                         }}, //

    // group 8
    { {0x0f, 0xba, 0b100},  { "bt",                 {{E,v},{I,b}}           }}, //
    { {0x0f, 0xba, 0b101},  { "bts",                {{E,v},{I,b}}           }}, //
    { {0x0f, 0xba, 0b110},  { "btr",                {{E,v},{I,b}}           }}, //
    { {0x0f, 0xba, 0b111},  { "btc",                {{E,v},{I,b}}           }}, //
    { {0x0f, 0xbb},         { "btc",                {{E,v},{G,v}}           }}, //
    { {0x0f, 0xbc},         { "bsf",                {{E,v},{G,v}}           }}, //
    { {0xf3, 0x0f, 0xbc},   { "tzcnt",              {{E,v},{G,v}}           }}, //
    { {0x0f, 0xbd},         { "bsr",                {{E,v},{G,v}}           }}, //
    { {0xf3, 0x0f, 0xbd},   { "lzcnt",              {{E,v},{G,v}}           }}, //
    { {0x0f, 0xbe},         { "movsx",              {{G,v},{E,b}}           }}, //
    { {0x0f, 0xbf},         { "movsx",              {{G,v},{E,w}}           }}, //
    
    { {0x0f, 0xc0},         { "xadd",               {{E,b},{G,b}}           }}, //
    { {0x0f, 0xc1},         { "xadd",               {{E,v},{G,v}}           }}, //
    // todo group 9 (cmpxchg16b)
    { {0x0f, 0xc8},         { "bswap",              {{Z,v}}                 }}, // w is ub
    { {0x0f, 0xc9},         { "bswap",              {{Z,v}}                 }}, // w is ub
    { {0x0f, 0xca},         { "bswap",              {{Z,v}}                 }}, // w is ub
    { {0x0f, 0xcb},         { "bswap",              {{Z,v}}                 }}, // w is ub
    { {0x0f, 0xcc},         { "bswap",              {{Z,v}}                 }}, // w is ub
    { {0x0f, 0xcd},         { "bswap",              {{Z,v}}                 }}, // w is ub
    { {0x0f, 0xce},         { "bswap",              {{Z,v}}                 }}, // w is ub
    { {0x0f, 0xcf},         { "bswap",              {{Z,v}}                 }}, // w is ub

    { {0x0f, 0xff},         { "ud0"                                         }},
};

// inline x64opinfo get_opinfo(vector<u8> opcode)
// {
//     auto it = std::ranges::find(x64opmap, opcode, &std::pair<vector<u8>, x64opinfo>::first);

//     if(it == std::end(x64opmap)) [[unlikely]]
//         return {};
//     else return it->second;
// }

inline x64opinfo get_opinfo(const vector<u8> opcode)
{
    if(!x64opmap.count(opcode)) [[unlikely]]
        return zero_x64opinfo;
    else
        return x64opmap.at(opcode);
    // https://gcc.gnu.org/bugzilla/show_bug.cgi?id=101361 ? only happens at O3/Ofast
}

}

#endif // SIM_TABLES64_H


