// o3 RISC Simulator
//
// uop helpers
//
// Lukas Heine 2021

#include "uops.hh"
// #include "core.hh"

// get unique ID for a register class
u8 getOpClassId(uop& op)
{
    switch(getOpPrefix(op))
    {
        default:
        case 0x0: // control
        case 0x1: // alu
            return regs_gp;
            break;
        case 0x2: // fpu
            return regs_fp;
            break;
        case 0x3: // vec alu
        case 0x4: // vec fpu
            return regs_vr;
            break;
    }
}

// get source and target regclass IDs, use is_cvt first
pair<u8, u8> getCvtClassIds(uop& op)
{
    // get first class as usual, second class is encoded in last opcode nibble
    // eg 00a3: regclass 0 -> regclass 3
    //    ^  ^
    return std::make_pair<u8, u8>(getOpClassId(op), (op.opcode & 0xf));
}

// get arf size for a register class
u16 getARFSize(uop& op)
{
    switch(getOpPrefix(op))
    {
        default:
        case 0x0: // control
        case 0x1: // alu
            return REGCLS_0_CNT;
            break;
        case 0x2: // fpu
            return REGCLS_1_CNT;
            break;
        case 0x3: // vec alu
        case 0x4: // vec fpu
            return REGCLS_2_CNT;
            break;
    }
}

// print uops as hexstring
std::ostream& operator<<(std::ostream& os, const uop& uop)
{
    os << hex_u<16> << uop.opcode  << " ";
    os << hex_u<16> << uop.control << " ";

    for(int j = 0; j < 4; j++)
        os << hex_u<8> << +uop.regs[j] << " ";

    os << hex_u<64> << uop.imm << "";

    return os;
}

// print readable decoded uop
std::stringstream uop_readable(const struct uop& uop)
{
    std::stringstream ss;

    ss << std::left << std::setw(10) << uopmap.at(uop.opcode).mnemonic;

    if(uop.control & use_cond) ss << "?u ";
    if(uop.control & set_cond) ss << "?s ";

    if(uop.control & use_ra)   ss << "a:r" << std::setw(4) << std::dec << +uop.regs[r_ra];
    if(uop.control & use_rb)   ss << "b:r" << std::setw(4) << std::dec << +uop.regs[r_rb];
    if(uop.control & use_rc)   ss << "c:r" << std::setw(4) << std::dec << +uop.regs[r_rc];
    if(uop.regs[r_rd])         ss << "d:r" << std::setw(4) << std::dec << +uop.regs[r_rd];
    if(uop.control & use_imm)  ss << "imm:0x" << hex_u<64> << +uop.imm;

    return ss;
}
