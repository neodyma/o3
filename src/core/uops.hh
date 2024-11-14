// o3 RISC Simulator
//
// uop decode (maybe..)
//
// Lukas Heine 2021

#ifndef SIM_UOPS_H
#define SIM_UOPS_H

#include "../types.hh"
#include "../util.hh"

#include "cconf.hh"

#include <map>

// uop metadata
struct uopinfo
{
    string      mnemonic;       // readable instruction name
    u8          ports;          // ports this uop can issue from (mask)
    u8          fu_type;        // FU type this uop runs on
    u16         ctrl_mask;      // allowed control bits (check or)
    u32         latency;        // execution latency
    string      description;    // uop description
    // i8          pl;            // required privilege level
}; // uopinfo

// bitmask for uop.control
// how to encode x64 high byte registers?..
// - use a single opcode bit for load/stores (reason `mov al <-> displ64`, one operand either lo/hi)
// - use highest 2 bits of imm otherwise since those are *never* needed for any non-control instructions on byte regs
typedef enum
{
    use_ra    = 0x0001, // used source operands
    use_rb    = 0x0002, // ..
    use_rc    = 0x0004, // ..
    use_imm   = 0x0008, // ..
    op_size   = 0x0070, // 3 bits: (1 << opsz) = operand bytes used, max 64 for now
    mop_first = 0x0080, // uop is first of macro instruction
    mop_last  = 0x0100, // uop is last of macro instruction
    imm_delay = 0x0200, // immediate will contain commit delay tbd
    rc_dest   = 0x0400, // use rc as second destination register
    data_type = 0x0800, // data type, vector: unset int / set float; .. tbd
    use_cond  = 0x1000, // uop uses last set condition
    set_cond  = 0x2000, // uop sets a new condition
    rd_extend = 0x4000, // sign/zero extend partial register writes
    rd_resize = 0x8000, // write back full register when opsz < regsz (e.g. dependence on high bytes)
} uop_ctrl;

typedef enum
{
    r_ra, r_rb, r_rc, r_rd,
} uop_regs;

typedef enum
{
    branch_none,
    branch_cond,
    branch_uncond,
} branch_type;

// first 4 bits of the opcode specify the op class, e.g. alu, fpu, vector, ..
constexpr u8  getOpPrefix(uop& op)     { return (u8) (op.opcode >> 12); }
constexpr u16 getOpCode(uop& op)       { return op.opcode & 0x0fff; }

// convert uops have dependeces across regfiles, this needs to be checked at issue
// no convert uops yet
constexpr u8  is_cvt(uop& op)          { return op.control & 0; }

constexpr u8  is_load(uop& op)
{ 
    u8 p = getOpPrefix(op);
    return (p == 0 || p == 2 || p == 3) && ((u8)op.opcode >= 0x20 && (u8)op.opcode < 0x28);
}

constexpr u8  is_store(uop& op)
{
    u8 p = getOpPrefix(op);
    // same as loads, opcodes x30 to x40
    return (p == 0 || p == 2 || p == 3) && ((u8)op.opcode >= 0x30 && (u8)op.opcode < 0x40);
}

constexpr u8  is_branch(uop& op)
{ 
    return (op.opcode >= 0x60 && op.opcode < 0x70) ? branch_uncond :
        (op.opcode >= 0x70 && op.opcode < 0x80) ? branch_cond :
        branch_none;
}

// extend these and ROBEntry.except if needed, for now only 16 bits error code are ever used in x64
// upper 16 bits = errorcode, lower 16 bits exception number
constexpr u32 setExcept(u16 e, u16 c) { return (((u32)c << 16) | e); }
constexpr u32 getExceptNum(u32 e)     { return e & 0xffff; }
constexpr u32 getExceptEC(u32 e)      { return e >> 16; }

constexpr u8  getOpSize(uop& op)      { return (1 << ((op.control & op_size) >> 4)); }
inline    u16 setOpSize(u16 opsz)     { return util::ld.at(opsz) << 4; }

constexpr u8  getImmDelay(uop& op)    { return byte(6, op.imm); } 

// lea/lda macros
constexpr u8  getLeaAdsz(uop& op)     { return byte(5, op.imm); }
constexpr u8  getLeaScale(uop& op)    { return byte(4, op.imm); }
constexpr i64 getLeaDispl(uop& op)    { return ((i64)(op.imm << 32)) >> 32; }


// condition register codes
typedef enum
{
    cc_CF = 0x0001,
    cc_PF = 0x0004,
    cc_AF = 0x0010,
    cc_ZF = 0x0040,
    cc_SF = 0x0080,
    cc_OF = 0x0800,
} uop_condition;

typedef enum
{
    scc_O, scc_NO, scc_B, scc_NB,
    scc_E, scc_NE, scc_BE, scc_NBE,
    scc_S, scc_NS, scc_P, scc_NP,
    scc_L, scc_NL, scc_LE, scc_NLE 
} cc_subcode;

// x64 style condition codes and subcodes
inline bool test_cc(u8 subcode, u64 ccreg)
{
    switch(subcode)
    {
        case scc_O:     return !!(ccreg & cc_OF);
        case scc_NO:    return  !(ccreg & cc_OF);
        case scc_B:     return !!(ccreg & cc_CF);
        case scc_NB:    return  !(ccreg & cc_CF);
        case scc_E:     return !!(ccreg & cc_ZF);
        case scc_NE:    return  !(ccreg & cc_ZF);
        case scc_BE:    return !!(ccreg & (cc_CF | cc_ZF));
        case scc_NBE:   return  !(ccreg & (cc_CF | cc_ZF));
        case scc_S:     return !!(ccreg & cc_SF);
        case scc_NS:    return  !(ccreg & cc_SF);
        case scc_P:     return !!(ccreg & cc_PF);
        case scc_NP:    return  !(ccreg & cc_PF);
        case scc_L:     return !!(ccreg & (cc_SF ^ cc_OF));
        case scc_NL:    return  !(ccreg & (cc_SF ^ cc_OF));
        case scc_LE:    return !!(ccreg & ((cc_SF ^ cc_OF) | cc_ZF));
        case scc_NLE:   return  !(ccreg & ((cc_SF ^ cc_OF) | cc_ZF));
        default:        return  0;
    }
}

// bitmask to select reservation station ports
typedef enum
{ // resize uopinfo.ports before adding more ports
    port0       = 0x01,
    port1       = 0x02,
    port2       = 0x04,
    port3       = 0x08,
    port4       = 0x10,
    port5       = 0x20,
    port6       = 0x40,
    port7       = 0x80,
    port_max    = port7,
} rs_port;

// bitmask to select functional units at ports
typedef enum
{
    port_ctrl   = port0 | port3,
    port_alu    = port0 | port1 | port2 | port3,
    port_agu    = port2 | port4 | port5 | port7,
    port_ld     = port4 | port5,
    port_st     = port6,
    port_brch   = port0 | port3,
    port_any    = 0xff,
} fu_ports;

typedef enum
{
    regs_gp,
    regs_fp,
    regs_vr,
} reg_class;

// fu identifier
typedef enum
{
    fu_any,
    fu_ctrl,
    fu_alu,
    fu_fpu,
    fu_vec,
    fu_ld,
    fu_st,
    fu_ldf,
    fu_stf,
    fu_ldv,
    fu_stv,
    fu_agu,
    fu_brch,
    fu_div,
    fu_mul,
} fu_type;

const std::string fu_type_str[fu_mul + 1] =
{
    ("any"), ("ctrl"), ("alu"), ("fpu"), ("vec"), ("ld"), ("st"),
    ("ldf"), ("stf"), ("ldv"), ("stv"), ("agu"), ("brnch"), ("div"), ("mul"),
};

// core exceptions
typedef enum
{
    ex_NONE     = 0x00, // no exception
    ex_UD       = 0x01, // undefined opcode
    ex_GP       = 0x02, // unspecified protection violation
    ex_PF       = 0x03, // page fault
    ex_REG      = 0x04, // invalid register reference
    ex_AV       = 0x05, // alignment violation
    ex_CTRL     = 0x06, // invalid uop control
    ex_BP       = 0x07, // breakpoint
    ex_HALT     = 0x08, // halt
    ex_DE       = 0x09, // divide error
    ex_UNSPEC   = 0x0a,
    ex_MAX,
} core_exceptions;

const std::string exception_str[ex_MAX] =
{
    ("none"), ("undefined opcode"), ("general protection violation"), ("page fault"), ("invalid register reference"),
    ("alignment violation"), ("invalid control"), ("breakpoint"), ("halt"), ("divide error"), 
    ("unspecified")
};

typedef enum
{
    expf_present  = 0x01,
    expf_write    = 0x02,
    expf_user     = 0x04,
    expf_reserved = 0x08,
    expf_ifetch   = 0x10,
} pagefault_bits;

u8           getOpClassId(uop& op);
pair<u8, u8> getCvtClassIds(uop& op);
u16          getARFSize(uop& op);

constexpr u16 classid_pair(u8 cls1, u8 cls2)
{
    return (((u16)cls1 << 8) | cls2);
}

constexpr u16 getRegSize(u8 futype)
{
    switch(futype)
    {
        default:
        case fu_ctrl:
        case fu_alu:
        case fu_ld:
        case fu_st:
            return REGCLS_0_SIZE;
            break;
        case fu_fpu:
        case fu_ldf:
        case fu_stf:
            return REGCLS_1_SIZE;
            break;
        case fu_vec:
        case fu_ldv:
        case fu_stv:
            return REGCLS_2_SIZE;
            break;
    }
}

typedef enum
{
    // control
    uop_nop       = 0x0000, // no-op
    uop_int       = 0x0010, // interrupt
    uop_rdtsc     = 0x0011, // timestamp
    uop_ld64      = 0x0020, // load
    uop_ld64h     = 0x0021, // load (high byte)
    uop_pop       = 0x0022, // pop
    uop_popx      = 0x0023, // popx
    uop_lda       = 0x0024, // load from complex address
    uop_lea       = 0x0028, // calculate address
    uop_st        = 0x0030, // store
    uop_push      = 0x0033, // push
    uop_pushx     = 0x0034, // push extended
    uop_move      = 0x0040, // copy reg to reg
    uop_copy2     = 0x0041, // copy reg,reg -> reg,reg
    uop_xchg      = 0x0045, // exchange registers
    uop_set       = 0x0048, // set reg to imm
    uop_movo      = 0x0050, // move conditional
    uop_movno     = 0x0051,
    uop_movb      = 0x0052,
    uop_movnb     = 0x0053,
    uop_movz      = 0x0054,
    uop_movnz     = 0x0055,
    uop_movbe     = 0x0056,
    uop_movnbe    = 0x0057,
    uop_movs      = 0x0058,
    uop_movns     = 0x0059,
    uop_movp      = 0x005a,
    uop_movnp     = 0x005b,
    uop_movl      = 0x005c,
    uop_movnl     = 0x005d,
    uop_movle     = 0x005e,
    uop_movnle    = 0x005f,
    uop_branch    = 0x0060, // jump unconditional
    uop_branchr   = 0x0061, // relative
    uop_branchrz  = 0x0062, // relative zero
    uop_brancho   = 0x0070, // relative conditional
    uop_branchno  = 0x0071,
    uop_branchb   = 0x0072,
    uop_branchnb  = 0x0073,
    uop_branchz   = 0x0074,
    uop_branchnz  = 0x0075,
    uop_branchbe  = 0x0076,
    uop_branchnbe = 0x0077,
    uop_branchs   = 0x0078,
    uop_branchns  = 0x0079,
    uop_branchp   = 0x007a,
    uop_branchnp  = 0x007b,
    uop_branchl   = 0x007c,
    uop_branchnl  = 0x007d,
    uop_branchle  = 0x007e,
    uop_branchnle = 0x007f,
    uop_setcond   = 0x0080, // set condition
    uop_cmc       = 0x0081, // complement carry
    uop_clc       = 0x0082, // clear carry
    uop_stc       = 0x0083, // set carry
    uop_cli       = 0x0084, // clear interrupt
    uop_sti       = 0x0085, // set interrupt
    uop_cld       = 0x0086, // clear direction
    uop_std       = 0x0087, // set direction
    
      
    // alu
    uop_nop_a     = 0x1000, // forced nop on ALU FU
    uop_add       = 0x1010, // add
    uop_adc       = 0x1011, // add + carry
    uop_sub       = 0x1012, // sub
    uop_sbb       = 0x1013, // sub + borrow
    uop_neg       = 0x1018, // negate value
    uop_mul       = 0x1020, // mul
    uop_imul      = 0x1024, // signed mul
    uop_div8      = 0x1028, // division ax->al/ah
    uop_divq      = 0x1029, // division quotient
    uop_divr      = 0x102a, // division remainder
    uop_idiv8     = 0x102b, // signed division ax->al/ah
    uop_idivq     = 0x102c, // signed division quotient
    uop_idivr     = 0x102d, // signed division remainder
    uop_lsl       = 0x1030, // << 
    uop_rsl       = 0x1031, // >>
    uop_rsa       = 0x1033, // sar
    uop_rol       = 0x1034, // rol
    uop_ror       = 0x1035, // ror
    uop_rcl       = 0x1036, // rcl
    uop_rcr       = 0x1037, // rcr
    uop_not       = 0x1040, // ~
    uop_and       = 0x1041, // &
    uop_or        = 0x1042, // |
    uop_xor       = 0x1043, // ^

    // fpu
    uop_nop_f     = 0x2000,
    uop_ld_f      = 0x2020, 
    uop_st_f      = 0x2030, 
    uop_set_f     = 0x2050,

    // vec
    uop_nop_v     = 0x3000,
    uop_ld_v      = 0x3020, 
    uop_ldu_v     = 0x3021,
    uop_st_v      = 0x3030, 
    uop_stu_v     = 0x3031,

    // vecf
    uop_nop_vecf  = 0x4000,
} uop_mnemo;

typedef enum
{
    px_invd,
    px_rip,
    px_flags,
} px_target;

// uop.opcode -> uopinfo
// todo privilege levels
// todo ctrl masks
// latencies for x64: use lowest latency for compute only uop, then adjust load/store to match tables
// todo issue latency is one lower than encoded here! latency =/= exec time!
const std::map<u16, uopinfo> uopmap
{ //  opcode      mnemonic      RS port     FU          ctrl    lat     description
    // control instructions
    { uop_nop,      { "nop",        port_any,   fu_any,     0x0181, 1,      "no operation"              }},
    { uop_int,      { "int",        port_ctrl,  fu_ctrl,    0xffff, 1,      "interrupt"                 }},
    { uop_rdtsc,    { "rdtsc",      port_ctrl,  fu_ctrl,    0xffff, 1,      "read timestamp"            }},
    { uop_ld64,     { "ld",         port_ld,    fu_ld,      0xffff, 1,      "load GP"                   }},
    { uop_ld64h,    { "ld",         port_ld,    fu_ld,      0xffff, 1,      "load GP"                   }}, // high byte
    { uop_pop,      { "pop",        port_ld,    fu_ld,      0xffff, 1,      "pop stack"                 }},
    { uop_popx,     { "popx",       port_ld,    fu_ld,      0xffff, 1,      "pop extended"              }},
    { uop_lda,      { "lda",        port_ld,    fu_ld,      0xffff, 1,      "load from eff. address"    }},
    { uop_lea,      { "lea",        port_agu,   fu_agu,     0xffff, 1,      "load effective address"    }},
    { uop_st,       { "st",         port_st,    fu_st,      0xffff, 1,      "store GP"                  }},
    { uop_push,     { "push",       port_st,    fu_st,      0xffff, 1,      "push stack"                }},
    { uop_pushx,    { "pushx",      port_st,    fu_st,      0xffff, 1,      "push extended"             }},
    { uop_move,     { "move",       port_alu,   fu_alu,     0xffff, 1,      "reg -> reg copy"           }},
    { uop_copy2,    { "copy2",      port_alu,   fu_alu,     0xffff, 1,      "reg,reg -> reg,reg copy"   }},
    { uop_xchg,     { "xchg",       port_alu,   fu_alu,     0xffff, 1,      "reg <-> reg swap"          }},
    { uop_set,      { "set",        port_alu,   fu_alu,     0xffff, 1,      "imm -> reg"                }},
    { uop_movo,     { "movo",       port_brch,  fu_brch,    0xffff, 1,      "conditional mov"           }},
    { uop_movno,    { "movno",      port_brch,  fu_brch,    0xffff, 1,      "conditional mov"           }},
    { uop_movb,     { "movb",       port_brch,  fu_brch,    0xffff, 1,      "conditional mov"           }},
    { uop_movnb,    { "movnb",      port_brch,  fu_brch,    0xffff, 1,      "conditional mov"           }},
    { uop_movz,     { "movz",       port_brch,  fu_brch,    0xffff, 1,      "conditional mov"           }},
    { uop_movnz,    { "movnz",      port_brch,  fu_brch,    0xffff, 1,      "conditional mov"           }},
    { uop_movbe,    { "movbe",      port_brch,  fu_brch,    0xffff, 1,      "conditional mov"           }},
    { uop_movnbe,   { "movnbe",     port_brch,  fu_brch,    0xffff, 1,      "conditional mov"           }},
    { uop_movs,     { "movs",       port_brch,  fu_brch,    0xffff, 1,      "conditional mov"           }},
    { uop_movns,    { "movns",      port_brch,  fu_brch,    0xffff, 1,      "conditional mov"           }},
    { uop_movp,     { "movp",       port_brch,  fu_brch,    0xffff, 1,      "conditional mov"           }},
    { uop_movnp,    { "movnp",      port_brch,  fu_brch,    0xffff, 1,      "conditional mov"           }},
    { uop_movl,     { "movl",       port_brch,  fu_brch,    0xffff, 1,      "conditional mov"           }},
    { uop_movnl,    { "movnl",      port_brch,  fu_brch,    0xffff, 1,      "conditional mov"           }},
    { uop_movle,    { "movle",      port_brch,  fu_brch,    0xffff, 1,      "conditional mov"           }},
    { uop_movnle,   { "movnle",     port_brch,  fu_brch,    0xffff, 1,      "conditional mov"           }},
    { uop_branch,   { "branch",     port_brch,  fu_brch,    0xffff, 1,      "unconditional branch"      }},
    { uop_branchr,  { "branchr",    port_brch,  fu_brch,    0xffff, 1,      "branch relative"           }},
    { uop_branchrz, { "branchrz",   port_brch,  fu_brch,    0xffff, 1,      "branch register zero"      }},
    { uop_brancho,  { "brancho",    port_brch,  fu_brch,    0xffff, 1,      "conditional branch"        }},
    { uop_branchno, { "branchno",   port_brch,  fu_brch,    0xffff, 1,      "conditional branch"        }},
    { uop_branchb,  { "branchb",    port_brch,  fu_brch,    0xffff, 1,      "conditional branch"        }},
    { uop_branchnb, { "branchnb",   port_brch,  fu_brch,    0xffff, 1,      "conditional branch"        }},
    { uop_branchz,  { "branchz",    port_brch,  fu_brch,    0xffff, 1,      "conditional branch"        }},
    { uop_branchnz, { "branchnz",   port_brch,  fu_brch,    0xffff, 1,      "conditional branch"        }},
    { uop_branchbe, { "branchbe",   port_brch,  fu_brch,    0xffff, 1,      "conditional branch"        }},
    { uop_branchnbe,{ "branchnbe",  port_brch,  fu_brch,    0xffff, 1,      "conditional branch"        }},
    { uop_branchs,  { "branchs",    port_brch,  fu_brch,    0xffff, 1,      "conditional branch"        }},
    { uop_branchns, { "branchns",   port_brch,  fu_brch,    0xffff, 1,      "conditional branch"        }},
    { uop_branchp,  { "branchp",    port_brch,  fu_brch,    0xffff, 1,      "conditional branch"        }},
    { uop_branchnp, { "branchnp",   port_brch,  fu_brch,    0xffff, 1,      "conditional branch"        }},
    { uop_branchl,  { "branchl",    port_brch,  fu_brch,    0xffff, 1,      "conditional branch"        }},
    { uop_branchnl, { "branchnl",   port_brch,  fu_brch,    0xffff, 1,      "conditional branch"        }},
    { uop_branchle, { "branchle",   port_brch,  fu_brch,    0xffff, 1,      "conditional branch"        }},
    { uop_branchnle,{ "branchnle",  port_brch,  fu_brch,    0xffff, 1,      "conditional branch"        }},
    { uop_setcond,  { "setcond",    port_ctrl,  fu_ctrl,    0xffff, 1,      "set condition register"    }},
    { uop_cmc,      { "cmc",        port_ctrl,  fu_ctrl,    0xffff, 1,      "complement carry flag"     }},
    { uop_clc,      { "clc",        port_ctrl,  fu_ctrl,    0xffff, 1,      "clear carry flag"          }},
    { uop_stc,      { "stc",        port_ctrl,  fu_ctrl,    0xffff, 1,      "set carry flag"            }},
    { uop_cld,      { "cld",        port_ctrl,  fu_ctrl,    0xffff, 1,      "clear direction flag"      }},
    { uop_std,      { "std",        port_ctrl,  fu_ctrl,    0xffff, 1,      "set direction flag"        }},


    // ALU instructions
    { uop_nop_a,    { "nop.a",      port_alu,   fu_alu,     0xffff, 1,      "no operation (ALU)"        }},
    { uop_add,      { "add",        port_alu,   fu_alu,     0xffff, 1,      "add"                       }},
    { uop_adc,      { "adc",        port_alu,   fu_alu,     0xffff, 1,      "add with carry"            }},
    { uop_sub,      { "sub",        port_alu,   fu_alu,     0xffff, 1,      "sub"                       }},
    { uop_sbb,      { "sbb",        port_alu,   fu_alu,     0xffff, 1,      "sub with borrow"           }},
    { uop_neg,      { "neg",        port_alu,   fu_alu,     0xffff, 1,      "negate two's complement"   }},
    { uop_mul,      { "mul",        port_alu,   fu_mul,     0xffff, 1,      "multiply"                  }},
    { uop_imul,     { "imul",       port_alu,   fu_mul,     0xffff, 3,      "signed multiply"           }},
    { uop_div8,     { "div8",       port_alu,   fu_div,     0xffff, 1,      "divide x->l/h"             }},
    { uop_divq,     { "divq",       port_alu,   fu_div,     0xffff, 1,      "division quotient"         }},
    { uop_divr,     { "divr",       port_alu,   fu_div,     0xffff, 1,      "division remainder"        }},
    { uop_idiv8,    { "idiv8",      port_alu,   fu_div,     0xffff, 1,      "signed divide x->l/h"      }},
    { uop_idivq,    { "idivq",      port_alu,   fu_div,     0xffff, 1,      "signed division quotient"  }},
    { uop_idivr,    { "idivr",      port_alu,   fu_div,     0xffff, 1,      "signed division remainder" }},
    { uop_lsl,      { "lsl",        port_alu,   fu_alu,     0xffff, 1,      "left shift logical"        }},
    { uop_rsl,      { "rsl",        port_alu,   fu_alu,     0xffff, 1,      "right shift logical"       }},
    { uop_rsa,      { "rsa",        port_alu,   fu_alu,     0xffff, 1,      "right shift arithmetic"    }},
    { uop_rol,      { "rol",        port_alu,   fu_alu,     0xffff, 1,      "rotate left"               }},
    { uop_ror,      { "ror",        port_alu,   fu_alu,     0xffff, 1,      "rotate right"              }},
    { uop_rcl,      { "rcl",        port_alu,   fu_alu,     0xffff, 1,      "rotate left with carry"    }},
    { uop_rcr,      { "rcr",        port_alu,   fu_alu,     0xffff, 1,      "rotate right with carry"   }},
    { uop_not,      { "not",        port_alu,   fu_alu,     0xffff, 1,      "logical negate"            }},
    { uop_and,      { "and",        port_alu,   fu_alu,     0xffff, 1,      "logical and"               }},
    { uop_or,       { "or",         port_alu,   fu_alu,     0xffff, 1,      "logical or"                }},
    { uop_xor,      { "xor",        port_alu,   fu_alu,     0xffff, 1,      "logical xor"               }},


    // FPU instructions
    { uop_nop_f,    { "nop.f",      port_any,   fu_fpu,     0xffff, 1,      "no operation (FPU)"        }},
    { uop_ld_f,     { "ld.f",       port_ctrl,  fu_ldf,     0xffff, 1,      "load FP"                   }},
    { uop_st_f,     { "st.f",       port_ctrl,  fu_stf,     0xffff, 1,      "store FP"                  }},
    { uop_set_f,    { "set.f",      port_ctrl,  fu_ctrl,    0xffff, 1,      "imm int -> FP"             }},

    // vector int
    { uop_nop_v,    { "nop.v",      port_any,   fu_vec,     0xffff, 1,      "no operation (vALU)"       }},
    { uop_ld_v,     { "ld.v",       port_ctrl,  fu_ldv,     0xffff, 1,      "load vec"                  }},
    { uop_ldu_v,    { "ldu.v",      port_ctrl,  fu_ldv,     0xffff, 1,      "load vec unaligned"        }},
    { uop_st_v,     { "st.v",       port_ctrl,  fu_stv,     0xffff, 1,      "store vec"                 }},
    { uop_stu_v,    { "stu.v",      port_ctrl,  fu_stv,     0xffff, 1,      "store vec unaligned"       }},

    // vector fp, not needed for the most part (control can encode int/fp type)
    { uop_nop_vecf, { "nop.vecf",   port_any,   fu_vec,     0xffff, 1,      "no operation (vFPU)"       }},

    // etc
    { 0xf000,       { "reserved",   port_ctrl,  fu_ctrl,    0xffff, 1,      "reserved"                  }},
}; // uopmap

static_assert((1 << (RS_PORTS-1)) == port_max);

// uop macros
// classic 4 op with signed accumulator and flag output
#define UOP_4OP_ACCI_GETF(instr)    (void)instr;

#define UOP_4OP_ACCU_GETF(instr)    (void)instr;

#endif // SIM_UOPS_H
