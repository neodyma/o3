// o3 RISC simulator
//
// frontends
//
// Lukas Heine 2021

#include "x64.hh"

x64Frontend::x64Frontend(MemoryManager& mmu, LatchQueue<uop>* uqueue,
        Simulator::SimulatorState& state) : Frontend(mmu, uqueue, state)
{
    // TODO make this static
    if(REGCLS_0_CNT < reg64_tmax)
        util::abort("x64 frontend requires at least", dec_u<0>, reg64_tmax, " GP registers.");
    if(REGCLS_2_CNT < reg64_tmmmax)
        util::abort("x64 frontend requires at least", dec_u<0>, reg64_tmmmax, " vec registers.");

    bp = new BTBPredictor();
    fetchbytes.resize(X64_FETCH_BYTES, 0);
    pdblocksz    = X64_FETCH_BYTES;
    pd_state     = pd_prefix;
    pd_remaining = 0;
    part_op      = zero_x64op;
    part_op.bytes.reserve(15);
    next_decoder = {};
    msrip        = 0;
    cur_tmp_gp   = reg64_t0 - 1;
    cur_tmp_vr   = reg64_tmm0 - 1;

    // util::log(0, x64def::get_opinfo({0xc3}) == x64def::zero_x64opinfo);

    util::log(LOG_FE_INIT, "x64 Frontend initialized with:");
    util::log(LOG_FE_INIT, "        Fetch block size: ", dec_u<0>, X64_FETCH_BYTES);
    util::log(LOG_FE_INIT, "        iQueue size:      ", dec_u<0>, IQUEUE_SIZE);
    util::log(LOG_FE_INIT, "        Decoders:         ", dec_u<0>, ds);
    util::log(LOG_FE_INIT, "");
}

u8 x64Frontend::cycle()
{
    fetch();
    udecode();

    util::log(LOG_64_PIPE1, "");
    return 0;
}

u8 x64Frontend::flush()
{
    util::log(LOG_64_PIPE1, "FE64:   Flushing all buffers.");   
    return flush(true);
}

u8 x64Frontend::flush(u8 total)
{
    // flush predecoder
    fetchbytes.resize(X64_FETCH_BYTES, 0);
    pdblocksz    = X64_FETCH_BYTES;
    pd_state     = pd_prefix;
    pd_remaining = 0;
    part_op      = zero_x64op;
    part_op.bytes.reserve(15);

    // flush everything else
    if(total)
    {
        next_decoder = {};
        msrip        = 0;
        cur_tmp_gp   = reg64_t0 - 1;
        cur_tmp_vr   = reg64_tmm0 - 1;
        iqueue.clear();
    }
    return 0;
}

// fetch instruction bundle from memory and predecode
u8 x64Frontend::fetch()
{
    // todo fetch traces instead of sequential ops? -> trace cache in bp
    // check status
    if( !(state.active & (if_active | pd_active)) )
    {
        util::log(LOG_64_PIPE1, "IFPD:   IFETCH/Predecode inactive.\n");
        return 1;
    }

    // worst case: bundle will contain 16 one byte instructions
    if(iqueue.size() >= (IQUEUE_SIZE - 16))
    {
        util::log(LOG_64_PIPE1, "IFPD: * Instruction queue is full, stalling frontend.");
        return 0;
    }

    // not every cycle is going to fetch new instructions
    if(state.active & if_active)
        util::log(LOG_64_PIPE1, "IFPD:   Fetching new instructions from memory.");


    auto parse_prefix = [&](u8& idx) {
        util::log(LOG_64_PIPE2, "          PD state PREFIX");
        for(;idx < pdblocksz;)
        {
            if(is_legacy(fetchbytes[idx]) || is_rex(fetchbytes[idx]))
            {
                util::log(LOG_64_PIPE3, "            Prefix ", hex_u<8>, +fetchbytes[idx], ".");
                switch(fetchbytes[idx])
                {   // group 1/2: last prefix counts
                    case 0xf0: case 0xf2: case 0xf3:
                        part_op.meta.has_g1 = fetchbytes[idx]; break;
                    case 0x64: case 0x65:
                        part_op.meta.has_g2 = fetchbytes[idx]; break;
                    // addr will make this a complex instruction (?)
                    case 0x66:
                        part_op.meta.has_66 = 1;
                        // part_op.meta.decoder = x64d_cmplx;
                        break;
                    case 0x67:
                        part_op.meta.has_67 = 1;
                        part_op.meta.decoder = x64d_cmplx;
                        break;
                    // todo pretty much all (implied) memory operations will go through complex decoder
                    // uarch p122/p160
                }

                part_op.bytes.push_back(fetchbytes[idx]);
                part_op.len++; idx++;
                part_op.off_opcode++;
            }
            else if(is_vex(fetchbytes[idx]) || is_evex(fetchbytes[idx]))
            {   // not supported -> #UD
                util::log(LOG_64_PIPE3, "            VEX/EVEX ", hex_u<8>, +fetchbytes[idx], " detected, #UD!");
                part_op  = zero_x64op;      // zero-op will always #UD
                pd_state = pd_reset;        // bypass instruction to iqueue
                idx      = pdblocksz; // and force out of range to stop predecoding
                state.active &= ~(if_active | pd_active);
                return 1;
            }
            else break;
        }

        // rex has to be the last prefix before esc/opcodes
        if((idx > 0) && is_rex(part_op.bytes[part_op.len-1]))
        {
            util::log(LOG_64_PIPE3, "            REX is valid.");
            part_op.meta.has_rex = 1;
            part_op.meta.off_rex = part_op.len-1;
        }
        return (idx < pdblocksz ? 0 : 1); // capture state if prefix byte is last in block
    };

    auto parse_opcode = [&](u8& idx) {
        util::log(LOG_64_PIPE2, "          PD state OPCODE");
        if(idx < pdblocksz && is_esc1(fetchbytes[idx]))
        {   // two or three byte opcode incoming
            util::log(LOG_64_PIPE3, "            Esc1 ", hex_u<8>, +fetchbytes[idx], ".");
            part_op.bytes.push_back(fetchbytes[idx]); // push back escape
            part_op.len++; idx++;
        }

        if(idx < pdblocksz && 
           is_esc1(part_op.bytes[part_op.len-1]) && is_esc2(fetchbytes[idx]))
        {   // two byte escape -> SSE3+, this will #UD for now (-> shut down fetch/pd)
            util::log(LOG_64_PIPE3, "            Esc2 ", hex_u<8>, +fetchbytes[idx], ", #UD!");
            part_op  = zero_x64op;
            pd_state = pd_reset;
            idx      = pdblocksz;
            state.active &= ~(if_active | pd_active);
            return 1;
        }

        if(idx >= pdblocksz) return 1; // escape was the last byte in fetch block

        // TODO macro ops <-> decoder type
        util::log(LOG_64_PIPE3, "            Opcode ", hex_u<8>, +fetchbytes[idx], ".");
        part_op.bytes.push_back(fetchbytes[idx]); // push back opcode
        part_op.len++; idx++;

        part_op.meta.op_mode = part_op.len - part_op.off_opcode - 1;
        return 0;
    };

    auto parse_modrm = [&](u8& idx) {
        util::log(LOG_64_PIPE2, "          PD state modRM");
        if(!part_op.off_modrm && idx < pdblocksz && use_modrm(part_op.bytes[part_op.len-1], part_op.meta.op_mode))
        {
            util::log(LOG_64_PIPE3, "            modR/M ", hex_u<8>, +fetchbytes[idx], ".");
            part_op.bytes.push_back(fetchbytes[idx]); // push back modrm
            part_op.off_modrm = part_op.len;
            part_op.len++; idx++;

            // at least one memory operand -> complex
            // (what about lea..?)
            if(modrm::get_mod(part_op.bytes.back()) != 0b11)
                part_op.meta.decoder = x64d_cmplx;
        }

        if(!part_op.off_sib && part_op.off_modrm && idx < pdblocksz && use_sib(part_op.bytes[part_op.off_modrm]))
        {
            util::log(LOG_64_PIPE3, "            SIB ", hex_u<8>, +fetchbytes[idx], ".");
            part_op.bytes.push_back(fetchbytes[idx]); // push back sib
            part_op.off_sib = part_op.len;
            part_op.len++; idx++;
        }

        // short circuit!
        if(part_op.off_modrm && (pd_remaining || (pd_remaining = get_displsz(part_op.bytes[part_op.off_modrm],
                (part_op.off_sib ? part_op.bytes[part_op.off_sib] : 0)))) && idx < pdblocksz)
        {
            std::vector<u8>::iterator start = fetchbytes.begin() + idx;

            // make sure we don't overflow out of fetch buffer when reading multiple bytes at once
            u8 bytecount = (pd_remaining <= (pdblocksz - idx) ? pd_remaining : (pdblocksz - idx));
            util::log(LOG_64_PIPE3, "            Displacement used.");

            part_op.bytes.insert(part_op.bytes.end(), start, start + bytecount); // push back displacement bytes
            
            if(!part_op.off_displ) part_op.off_displ = part_op.len; // don't overwrite offset if already set
            part_op.len  += bytecount;
            pd_remaining -= bytecount;
            idx          += bytecount;
        }

        // modrm, displ or sib not finished
        // these checks are getting to complex... might have to rewrite the predecoder if everything else is done 
        return ((pd_remaining ||
            (!part_op.off_modrm && use_modrm(part_op.bytes[part_op.off_opcode], part_op.meta.op_mode)) || 
            (!part_op.off_sib && part_op.off_modrm && use_sib(part_op.bytes[part_op.off_modrm]))) ? 1 : 0);
    };

    auto parse_imm = [&](u8& idx) {
        util::log(LOG_64_PIPE2, "          PD state IMM");
        u8 has_rex_w = part_op.meta.has_rex && !!(part_op.bytes[part_op.meta.off_rex] & rex::w);
        u8 has_66_67 = part_op.meta.has_66 || part_op.meta.has_67;

        u8 opsz = (has_rex_w ? 8 : (has_66_67 ? 2 : 4));

        // ....
        if(pd_remaining || (pd_remaining = get_immsz(part_op.bytes[part_op.off_opcode + part_op.meta.op_mode], opsz,
            part_op.meta.op_mode, part_op.off_modrm ? modrm::get_reg(part_op.bytes[part_op.off_modrm]) : 0)))
        {
            if(idx >= pdblocksz) return 1; // we need an immediate but it's not in the current buffer
            
            std::vector<u8>::iterator start = fetchbytes.begin() + idx;

            u8 bytecount = (pd_remaining <= (pdblocksz - idx) ? pd_remaining : (pdblocksz - idx));
            util::log(LOG_64_PIPE3, "            Immediate used.");

            part_op.bytes.insert(part_op.bytes.end(), start, start + bytecount); // push back immediate bytes

            if(!part_op.off_imm) part_op.off_imm = part_op.len;
            // util::log(0, "partoplen ", dec_u<0>, +part_op.len);
            // part_op.len  += pd_remaining;
            part_op.len  += bytecount;
            // util::log(0, "partoplen ", dec_u<0>, +part_op.len);
            // getchar();
            pd_remaining -= bytecount;
            idx          += bytecount;
        }

        // all instruction bytes read, reset
        if(!pd_remaining)
        {
            util::log(LOG_64_PIPE2, "IFPD:     Instruction decoded, state reset.");
            pd_state = pd_reset;
        }
        else
            util::log(LOG_64_PIPE2, "              ", +pd_remaining, " bytes left.");
        return 0;
    };

    // parse one instruction starting from fetch block index, set index to start of next instruction
    auto parse_instr = [&](u8& idx) {
        switch(pd_state)
        {   // no errors:   fall through until done 
            // error?       capture last state for next predecode cycle
            case pd_reset:
            case pd_prefix:
                pd_state = pd_prefix;
                if(parse_prefix(idx)) break;
                [[fallthrough]]; // prefix will return error code on fast #UD!
            case pd_opcode:
                pd_state = pd_opcode;
                if(parse_opcode(idx)) break;
                [[fallthrough]]; // same for opcode
            case pd_modrm: 
                pd_state = pd_modrm;
                if(parse_modrm(idx)) break;
                [[fallthrough]];
            case pd_imm:   
                pd_state = pd_imm;   
                parse_imm(idx);
        }
    };

    // check iQueue size here

    // read bytes into buffer
    // split up macro ops and put into iqueue
    // fuse ops inside the macro queue
    // take out ops from iqueue and decode them to uops
    // buffer uops for loop detection and micro fusion?


    util::log(LOG_64_PIPE1, "IFPD:   Fetchaddr: ", hex_u<64>, fetchaddr);

    u64 fetchbase = fetchaddr & X64_FETCH_ALIGN;  // address of the fetch block
    u64 fetchoffs = fetchaddr & ~X64_FETCH_ALIGN; // index of instructions we are actually interested in
    u64 bytesread = 0;
    
    // if a page fault occurs, add instruction with length -1 to iqueue
    // decode will directly translate it to uop_int #PF(EC)
    u8  inject_pf = 0;

    util::log(LOG_64_PIPE1, "IFPD:   Base: ", hex_u<64>, fetchbase, ". Offs: ", hex_u<X64_FETCH_BYTES/8>, fetchoffs, ".");

    try
    {
        if((state.active & if_active))
            if(!mmu.is_busy(fetchbase, X64_FETCH_BYTES))
                bytesread = mmu.read(fetchbase, fetchbytes.data(), X64_FETCH_BYTES, MM::p_x).second;
            else
                util::log(LOG_64_PIPE1, "IFPD:   Waiting for memory ...");
        else
            util::log(LOG_64_PIPE1, "IFPD:   Predecoding in progress, not fetching anything.");
    }
    catch(const InvalidAddrException& ia)
    {
        state.active &= ~(if_active | pd_active);

        if(SILENT_HALT)
            util::log(LOG_64_PIPE1, "IFPD:   End of code reached.");
        else
            inject_pf = part_op.bytes.empty() ? 1 : 2; // inject #PF after the current instruction, since this one is ok
    }
    catch(const ProtectionViolationException& pv)
    {
        util::log(LOG_64_PIPE1, "IFPD:   Fetch ", pv.what(), " Injecting #PF.");
        inject_pf = 1;
    }
    catch(const PageNotMappedException& pm)
    {
        util::log(LOG_64_PIPE1, "IFPD:   Fetch ", pm.what(), " Injecting #PF.");
        inject_pf = 1;
    }


    try
    {
        // last bytes in byte vector are invalid and should not be treated as instruction bytes
        // this marks the end of code -> shut down fetch/pd
        if(bytesread < X64_FETCH_BYTES)
        {
            util::log(LOG_64_PIPE1, "IFPD:   End of code reached.");
            fetchbytes.resize(bytesread); // reset this on flush!!
            pdblocksz     = bytesread;
            state.active &= ~(if_active | pd_active);
            // don't return here, buffer is not empty yet
        }

        util::log(LOG_64_BUF, "\nIFPD:   Predecode buffer: ", (fetchbytes.size() ? fetchbytes : (vector<u8>)0 ));

        if(!part_op.bytes.empty())
            util::log(LOG_64_BUF, "\nIFPD:   Instruction buffer: ", part_op, "\n");

        // do some predecoding:
        // instruction length   (max 15)
        // offsets:
        //   prefixes
        //   opcode
        //   modrm
        //   sib
        //   displacement
        //   immediate
        // don't check for #UD or anything else yet! decoders will do that

        for(u8 i = fetchoffs; /*i < pdblocksz*/;)
        {
            util::log(LOG_64_PIPE2, "\nIFPD:   Predecoding ...");
            parse_instr(i);

            // instruction not finished but reached end of block
            if(pd_state != pd_reset)
            {
                // instruction continues in next block
                fetchaddr = fetchbase + X64_FETCH_BYTES;
                util::log(LOG_64_PIPE2, "IFPD:   Instruction incomplete, fetching next block.");
                util::log(LOG_64_PIPE2, "          ", part_op);

                if(!inject_pf)
                    break;
            }

            util::log(LOG_64_PIPE1, "IFPD:   Predecode yielded: ", part_op);

            u64 pred = 0, seq = 0;
            // slightly more complex than risc prediction since we might have to jump from inside the fetch block
            // we don't really have to "predict" unconditional jumps, but the predictor may already know the target
            // todo
            seq = state.in_flight.back() + part_op.bytes.size();
            util::log(LOG_64_PIPE3, "          sequential rip ", hex_u<64>, seq, " -> seq_addrs");
            state.seq_addrs.push_back(seq);

            if(u8 brtype; (brtype = is_branch(part_op)))
                pred = bp->predict(fetchaddr, seq, -1);
            else [[likely]]
                pred = seq;

            if(inject_pf == 1) // page fault on this instruction
                part_op.len = 0xff;
            else if(inject_pf == 2) // next instruction
                inject_pf--;

            iqueue.push_back((state.cycle + FETCH_LATENCY), part_op);
            part_op = zero_x64op;

            util::log(LOG_64_PIPE2, "IFPD:   Instruction at v.", hex_u<64>, state.in_flight.back(), " added. ",
                "Sequential instruction at v. ", hex_u<64>, seq);
            // getchar();

            // need to fetch next instruction from somewhere else
            if(pred != seq)
            {
                fetchaddr = pred;
                state.active |= (if_active | pd_active);
                // fetchbytes.resize(X64_FETCH_BYTES); // !!!!!!!
                flush(false);
                util::log(LOG_64_PIPE3, "          predicted rip ", hex_u<64>, pred, " -> in_flight");
                state.in_flight.push_back(pred);
                break;
            }
            else fetchaddr = seq;

            util::log(LOG_64_PIPE3, "          next rip ", hex_u<64>, pred, " -> in_flight");
            state.in_flight.push_back(pred);

            if(inject_pf) break;
        }

        if(!pdblocksz && inject_pf)
        {
            part_op.len = 0xff;
            iqueue.push_back((state.cycle + 1), part_op);
            // u64 seq = (state.seq_addrs.empty() ? state.in_flight.back() : state.seq_addrs.back()) + part_op.bytes.size();
            util::log(LOG_64_PIPE3, "ifseq pushing back ", hex_u<64>, fetchaddr);
            state.seq_addrs.push_back(fetchaddr);
            state.in_flight.push_back(fetchaddr);
        }

        if(iqueue.size()) 
        {
            util::log(LOG_64_BUF, "\nIFPD:   Instruction Queue:");
            for(u8 i = 0; i < iqueue.size(); i++)
                util::log(LOG_64_BUF, dec_u_lf<2>, +i, " |    ", iqueue.at(UINT64_MAX, i));
            util::log(LOG_64_BUF, "");
        }

    }
    catch(const std::exception& e)
    {
        util::log(0, e.what());    
    }


    return 0;
}

// try to fuse macro instructions
// disabled for now
u8 x64Frontend::fuse_macro()
{
    return 0;
}

// pass instructions to decoders
u8 x64Frontend::udecode()
{
    if( !(state.active & de_active) )
    {
        util::log(LOG_64_PIPE1, "DE__:   Macro decode inactive.\n");
        return 1;
    }

    // try to assign instruction at iqueue head to available decoder
    // todo exhaustive search
    for(u8 i = 0; i < iqueue.size(); i++)
        for(auto& dec : ds.decoders)
            if(iqueue.ready(state.cycle) && !dec.busy && (dec.type == iqueue.front(state.cycle).meta.decoder))
            {   
                util::log(LOG_64_PIPE2, "DE__:   Matching decoder found: ", +dec.id, " ", x64d_type_str[dec.type]);
                next_decoder.push_back(dec.id);
                dec.instr = iqueue.get_front(state.cycle);
                dec.busy  = 1;
                if(iqueue.empty()) break;
            }

    // decode instructions on assigned decoders
    x64Decoder* next = next_decoder.empty() ? nullptr : &ds.decoders[next_decoder.front()];
    for(;next && next->busy;)
    {
        // worst case: uop bundle contains 4 instructions
        if(uqueue->size() >= (UQUEUE_SIZE - 4))
        {
            util::log(LOG_64_PIPE1, "DE__: * uQ might overflow. Stalling macro decode.");
            break;
        }

        util::log(LOG_64_PIPE1, "DE.", +next->id, ":   Decoding macro op ", next->instr);
        if(!run_decode(next->instr))
        {   // decoder is finished, remove from queue
            next->busy  = 0;
            next->instr = zero_x64op;
            next_decoder.pop_front();
            next = next_decoder.empty() ? nullptr : &ds.decoders[next_decoder.front()];
        }
        // empty out buffer here
        // and break if uqueue is full
    }

    if(iqueue.empty() && !next && !(state.active & (if_active | pd_active)))
        state.active &= ~de_active;

    return 0;
}

// fuse micro ops inside the uQ
// disabled for now
u8 x64Frontend::fuse_micro()
{
    return 0;
}

// print x64op as hexstring
std::ostream& operator<<(std::ostream& os, const x64op& op)
{
    if(op.bytes.empty())
        return os << "|| __ ||";

    os << (op.off_opcode != 0 ? "||p " : "||o ");

    for(u8 i = 0; i < op.bytes.size(); i++)
    {
        if(op.off_opcode && i == op.off_opcode) os << "|o ";
        if(op.off_modrm  && i == op.off_modrm)  os << "|m ";
        if(op.off_sib    && i == op.off_sib)    os << "|s ";
        if(op.off_displ  && i == op.off_displ)  os << "|d ";
        if(op.off_imm    && i == op.off_imm)    os << "|i ";
        os << hex_u<8> << +op.bytes[i] << " ";
    }

    return os << "||";
}

std::ostream& operator<<(std::ostream& os, const DecoderStation& ds)
{
    os << "||";

    for(auto dec : ds.decoders)
        os << " " << x64d_type_str[dec.type] << " |";

    return os << "|";
}

// get a temporary register from the register pool
// for now this is implemented as ring buffer (use free/use list instead?)
// todo those registers need to be reset
u8 x64Frontend::get_tmpreg(const u8 regcls)
{
    switch(regcls)
    {
        default:
        case regs_gp: // gp
            return (cur_tmp_gp = reg64_t0 + ((cur_tmp_gp - reg64_t0 + 1) % (reg64_tmax - reg64_t0 + 1)));
            break;
        case regs_vr: // vr
            return (cur_tmp_vr = reg64_tmm0 + ((cur_tmp_vr - reg64_tmm0 + 1) % reg64_tmmmax - reg64_tmm0 + 1));
            break;
    }
}

// decode x64ops to uops and place them into the uqueue
// - transform modrm/displ into usable uop src/dest
// -- no modrm/modrm.11: only register operands
// -- modrm (+sib): ld/op or op/st or ld/op/st uop bundle
u8 x64Frontend::run_decode(const x64op& op)
{
    // uops to be added to the uqueue
    vector<uop> uops = {};
    uop ud           = { uop_int, use_imm, {0}, ex_UD };

    // treat the entire macro op as #ud
    auto raise_ud = [&]() {
        uops.clear();
        ud.control |= mop_first | mop_last;
        util::log(LOG_64_PIPE1, "DE__:   Undefined instruction.");
        uqueue->push_back(state.cycle + DECODE_LATENCY, ud);
    };

    // injected page fault
    if(op.len == 0xff)
    {
        uops.clear();
        util::log(LOG_64_PIPE1, "DE__:   Page Fault injected.");
        uqueue->push_back(state.cycle + DECODE_LATENCY, { uop_int,
            (mop_first | mop_last | use_imm),
            { 0 },
            // todo present bit
            setExcept(ex_PF, (/*expf_present |*/  expf_ifetch | expf_user))
        });
        return 0;

    }
    // should #UD (e.g. unsupported isa extension is set to len 0)
    else if(op.len == 0 || op.len > 15) [[unlikely]]
    {
        util::log(LOG_64_PIPE1, "DE__:   Invalid length. ", dec_u<0>, +op.len);
        raise_ud();
        return 0;
    }

    const u8 opcode  = op.bytes[op.off_opcode + op.meta.op_mode]; // main opcode
    const u8 segbase = to_ureg((op.meta.has_g2 == 0x64 ? reg64_fsbase : reg64_gsbase), op.meta.has_g2); // ureg(0/fs/gs)
    const u8 modrm   = op.off_modrm    ? op.bytes[op.off_modrm]    : 0; // modrm byte
    const u8 sib     = op.off_sib      ? op.bytes[op.off_sib]      : 0; // sib byte
    const u8 rex     = op.meta.has_rex ? op.bytes[op.meta.off_rex] : 0; // rex prefix
    u8 mod_reg       = 0; // extracted modrm.reg (+ rex.r)
    u8 sib_idx       = 0; // extracted sib index (+ rex.x)
    u8 sib_scl       = 0; // extracted scale
    u8 rexb_ex       = 0; // rex.b: either modrm.rm, sib.b, or opcode register
    u8 sib_useb      = 0; // sib base valid? (use for [*] column)
    u8 sib_usei      = 0; // sib index valid? (0b100 -> base only) 
    i64 displ        = 0; // signed(!) displacement

    // parse fields dependent on modrm
    if(op.off_modrm)
    {
        // modrm = op.bytes[op.off_modrm];

        // mod_reg = (op.meta.has_rex ? ((op.bytes[op.meta.off_rex] & rex::r) << 1) : 0)
        //     | modrm::get_reg(modrm);

        mod_reg = (rex ? ((rex & rex::r) << 1) : 0) | modrm::get_reg(modrm);

        sib_idx = (rex ? ((rex & rex::x) << 2) : 0) | (op.off_sib ? sib::get_i(sib) : 0);
        
        // index invalid if equal to 0b100
        if((sib_idx & 0b111) == 0b100)
            sib_idx = 0;

        // no sib                 -> extend modrm.rm
        // sib used               -> extend sib.b, careful! 'rbp / no base' depends on modrm.mod
        // reg operand (no .x/.r) -> extend opcode register field (done after parsing below)
        rexb_ex = (rex ? ((rex & rex::b) << 3) : 0) | (op.off_sib ? sib::get_b(sib) : modrm::get_rm(modrm));

        if(op.off_sib)
        {
            sib_scl = (1 << sib::get_s(sib));
            // column =/= 0b101; or modrm valid
            sib_useb = !(sib::get_b(sib) == 0b101) || (modrm::get_mod(modrm) == 0b01 || modrm::get_mod(modrm) == 0b10);

            sib_usei = !(sib::get_i(sib) == 0b100);
        }

        // displacement will be from 0-8 bytes!
        displ            = 0;
        size_t displ_len = ((op.off_imm ? op.off_imm : op.bytes.size()) - op.off_displ);
        // get usable sign extended displacement from instruction bytes
        if(op.off_displ)
            std::memcpy(&displ, &op.bytes[op.off_displ], displ_len);

        displ = (displ << ((8 - displ_len)*8)) >> ((8 - displ_len)*8);
    }

    u64 imm = 0;
    size_t immbytes = 0;
    if(op.off_imm)
    {
        immbytes = (op.bytes.size() - op.off_imm);
        std::memcpy(&imm, &op.bytes[op.off_imm], immbytes);
    }

    u8 has_rex_w = rex && (rex & rex::w);

    u8 opsz = (has_rex_w ? 8 : (op.meta.has_66 ? 2 : 4)); // exec operand size
    u8 adsz = (op.meta.has_67 ? 4 : 8);                   // address size
    u8 ldsz = 0;                                          // size for auxiliary loads/stores

    const u8 opgrp  = get_group(opcode, op.meta.op_mode);
    const u8 reqpfx = has_reqpfx(opcode, op.meta.op_mode);
    const u8 haspfx = op.meta.has_g1 ? op.meta.has_g1 : 0;

    vector<u8> opvec = [&]{
        vector<u8> v = {};
        if(reqpfx && haspfx)     v.push_back(haspfx);
        if(op.meta.op_mode == 1) v.push_back(0x0f);
        v.push_back(opcode);
        if(opgrp)                v.push_back(modrm::get_reg(modrm)); // only 3 bit extension for now
        return v;
    }();

    x64def::x64opinfo opinfo            = x64def::get_opinfo(opvec);
    vector<x64def::x64operand> operands = opinfo.operands;

    util::log(LOG_64_PIPE1, "        macro mnemonic: ", opinfo.mnemonic);

    // register operand
    if(modrm::get_mod(modrm) == 0b11)
    {
        util::log(LOG_64_PIPE3, "          Register operands used.");
        util::log(LOG_64_PIPE3, "            ARF: reg r", dec_u<0>, +to_ureg(mod_reg));
        util::log(LOG_64_PIPE3, "            ARF: r/m r", dec_u<0>, +to_ureg(rexb_ex));
    }
    // memory operand, no sib
    else if(!op.off_sib)
    {
        util::log(LOG_64_PIPE3, "          Memory operand used.");
        util::log(LOG_64_PIPE3, "            ARF: reg r", dec_u<0>, +to_ureg(mod_reg));
        util::log(LOG_64_PIPE3, "            ARF: r/m r", dec_u<0>, +to_ureg(rexb_ex));
        util::log(LOG_64_PIPE3, "            Displ: ", hex_u<32>, displ ? displ : imm ? imm : 0);
    }
    // memory operand, sib
    else 
    {
        util::log(LOG_64_PIPE3, "          SIB operand used.");
        util::log(LOG_64_PIPE3, "            ARF: reg r ", dec_u<0>, +to_ureg(mod_reg));
        util::log(LOG_64_PIPE3, "            ARF: sib b ", dec_u<0>, +to_ureg(rexb_ex, sib_useb));
        util::log(LOG_64_PIPE3, "            ARF: sib i ", dec_u<0>, +to_ureg(sib_idx));
        util::log(LOG_64_PIPE3, "            ARF: seg b ", dec_u<0>, +segbase);
        util::log(LOG_64_PIPE3, "            Scale: ", dec_u<0>, +sib_scl);
        util::log(LOG_64_PIPE3, "            Displ: ", hex_u<32>, displ ? displ : 0);
    }

    // auxiliary archregs for results not written to 'main' registers
    // TODO refactor names
    u8  load_reg = 0; // load target
    u8  addr_reg = 0; // store address
    u8  storeimm = 0; // store immediate with implicit store
    u8  opsrc    = 0; // compute source
    u8  opdst    = 0; // compute target
    u8  temp_op  = 0; // compute target is temporary (memory destination)
    u16 extflag  = 0; // resize or extend

    // encode ah..bh as source and/or destination
    const u64 src_rh = ((u64)1 << 63); // high byte reg in dst
    const u64 dst_rh = ((u64)1 << 62); // high byte reg in src

    // set up operands for GP instructions
    // some of this can be reused for SSE
    if(is_gp(op))
    {
        // 'compute' uop needs the target operand size (which might be different from load size)
        // first operand will tell 'direction'
        if(!operands.empty())
        {
            opsz = get_opsz(opsz, operands[0].operand_type);

            // sign extend immediates to operand size
            if(x64def::is_immop(operands.back()))
                imm = sx(imm, immbytes, opsz);

            // high byte register used: opsz 1b, no rex, register in range
            if(x64def::is_rmop(operands[0]))
            {
                if((opsz == 1) && !rex && op.off_modrm && (modrm::get_mod(modrm) == 0b11) &&
                  (rexb_ex >= reg64_sp && rexb_ex <= reg64_di))
                {
                    opdst = rexb_ex - reg64_sp;
                    imm |= dst_rh; // this does never conflict with sign extended immediate!
                }
                else [[likely]]
                    opdst = rexb_ex;
            }
            else
            {
                if((opsz == 1) && !rex && (mod_reg >= reg64_sp && mod_reg <= reg64_di))
                {   
                    opdst = mod_reg - reg64_sp;
                    imm |= dst_rh;
                }
                else [[likely]]
                    opdst = mod_reg;
            }

            // if(x64def::is_rmop(operands.back()))
            if(x64def::get_rmop(operands).second != 0)
            {
                if((opsz == 1) && !rex && op.off_modrm && (modrm::get_mod(modrm) == 0b11) &&
                  (rexb_ex >= reg64_sp && rexb_ex <= reg64_di))
                {
                    opsrc = rexb_ex - reg64_sp;
                    imm |= src_rh;
                }
                else [[likely]]
                    opsrc = rexb_ex;
            }
            else
            {
                if((opsz == 1) && !rex && (mod_reg >= reg64_sp && mod_reg <= reg64_di))
                {
                    opsrc = mod_reg - reg64_sp;
                    imm |= src_rh;
                }
                else [[likely]] 
                    opsrc = mod_reg;
            }

            extflag = (opsz == 4) ? rd_extend : rd_resize;

            // src == dst in this case, respect high byte regs
            if((operands.size() == 1) && (opsz == 1))
                imm |= (imm & dst_rh ? src_rh : 0);
        }

        // find the memory operand (if used) and load into aux source
        // some uops don't need the loaded value (e.g. mov), naive solution: just remove the load uop in decode below
        if(!operands.empty() && op.off_modrm && (modrm::get_mod(modrm) != 0b11))
        {
            // one byte opcodes will always use gp regs (ignore ESC/FPU..)
            load_reg = get_tmpreg(regs_gp);
            ldsz = get_opsz(opsz, x64def::get_rmop(operands).first.operand_type);

            // TODO somehow encode address size? (probably in highest word of imm..)
            // operands:
            // - ra: either extended sib.b or modrm.rm
            // - rb: sib index if used
            // - rc: segment base if used
            // - im: scale | displacement
            uops.push_back({ uop_lda,
                (u16)(setOpSize(ldsz) | use_ra | use_rb | use_rc | use_imm),
                // base is invalid for disp32 only (rip-relative)
                { to_ureg(rexb_ex, (sib_useb | (!op.off_sib && modrm::get_rm(modrm) != 0b101))),
                    to_ureg(sib_idx, sib_usei), segbase, to_ureg(load_reg) },
                ( (imm & (bitmask(2) << 62)) | ((u64)adsz << 40) | ((u64)sib_scl << 32) | (bitmask(32) & (i32)displ)) });

            // destination in memory -> compute target is probably temporary
            if(x64def::is_rmop(operands[0]))
            {
                opdst  = get_tmpreg(regs_gp);
                temp_op = 1; // no source dependence on temporary register!
            }
        }
        // else we don't need to load anything
    }


    if(op.meta.op_mode == 0) // one byte opcodes
    {
        // TODO zero idioms?
        switch(opcode)
        {
            default: [[unlikely]] // invalid opcodes are _not_ detected by predecode!
                raise_ud();
                break;

            case 0x00 ... 0x03: // add: Eb/Gb, Ev/Gv, Gb/Eb, Gv/Ev
                uops.push_back({ uop_add,
                    (u16)(setOpSize(opsz) | set_cond | extflag | use_ra | use_rb | use_rc),
                    { to_ureg(opdst, !temp_op), to_ureg(opsrc), to_ureg(load_reg, load_reg), to_ureg(opdst) },
                    imm });
                break;


            case 0x04 ... 0x05: // add: Zb/Ib, Zv/Iz (Z -> a)
                uops.push_back({ uop_add,
                    (u16)(setOpSize(opsz) | set_cond | extflag | use_ra | use_imm),
                    { to_ureg(reg64_a), 0, 0, to_ureg(reg64_a) },
                    imm });
                break;


            case 0x08 ... 0x0b: // or: Eb/Gb, Ev/Gv, Gb/Eb, Gv/Ev
                uops.push_back({ uop_or,
                    (u16)(setOpSize(opsz) | set_cond | extflag | use_ra | use_rb | use_rc),
                    { to_ureg(opdst, !temp_op), to_ureg(opsrc), to_ureg(load_reg, load_reg), to_ureg(opdst) },
                    imm });
                break;


            case 0x0c ... 0x0d: // or: Zb/Ib, Zv/Iz (Z -> a)
                uops.push_back({ uop_or,
                    (u16)(setOpSize(opsz) | set_cond | extflag | use_ra | use_imm),
                    { to_ureg(reg64_a), 0, 0, to_ureg(reg64_a) },
                    imm });
                break;


            case 0x10 ... 0x13: // adc: Eb/Gb, Ev/Gv, Gb/Eb, Gv/Ev
                uops.push_back({ uop_adc,
                    (u16)(setOpSize(opsz) | use_cond | set_cond | extflag | use_ra | use_rb | use_rc),
                    { to_ureg(opdst, !temp_op), to_ureg(opsrc), to_ureg(load_reg, load_reg), to_ureg(opdst) },
                    imm });
                break;


            case 0x14 ... 0x15: // adc: Zb/Ib, Zv/Iz (Z -> a)
                uops.push_back({ uop_adc,
                    (u16)(setOpSize(opsz) | use_cond | set_cond | extflag | use_ra | use_imm),
                    { to_ureg(reg64_a), 0, 0, to_ureg(reg64_a) },
                    imm });
                break;


            case 0x18 ... 0x1b: // sbb: Eb/Gb, Ev/Gv, Gb/Eb, Gv/Ev
                uops.push_back({ uop_sbb,
                    (u16)(setOpSize(opsz) | use_cond | set_cond | extflag | use_ra | use_rb | use_rc),
                    { to_ureg(opdst, !temp_op), to_ureg(opsrc), to_ureg(load_reg, load_reg), to_ureg(opdst) },
                    imm });
                break;


            case 0x1c ... 0x1d: // sbb: Zb/Ib, Zv/Iz (Z -> a)
                uops.push_back({ uop_sbb,
                    (u16)(setOpSize(opsz) | use_cond | set_cond | extflag | use_ra | use_imm),
                    { to_ureg(reg64_a), 0, 0, to_ureg(reg64_a) },
                    imm });
                break;


            case 0x20 ... 0x23: // and: Eb/Gb, Ev/Gv, Gb/Eb, Gv/Ev
                uops.push_back({ uop_and,
                    (u16)(setOpSize(opsz) | set_cond | extflag | use_ra | use_rb | use_rc),
                    { to_ureg(opdst, !temp_op), to_ureg(opsrc), to_ureg(load_reg, load_reg), to_ureg(opdst) },
                    imm });
                break;


            case 0x24 ... 0x25: // and: Zb/Ib, Zv/Iz (Z -> a)
                uops.push_back({ uop_and,
                    (u16)(setOpSize(opsz) | set_cond | extflag | use_ra | use_imm),
                    { to_ureg(reg64_a), 0, 0, to_ureg(reg64_a) },
                    imm });
                break;


            case 0x28 ... 0x2b: // sub: Eb/Gb, Ev/Gv, Gb/Eb, Gv/Ev
                uops.push_back({ uop_sub,
                    (u16)(setOpSize(opsz) | set_cond | extflag | use_ra | use_rb | use_rc),
                    { to_ureg(opdst, !temp_op), to_ureg(opsrc), to_ureg(load_reg, load_reg), to_ureg(opdst) },
                    imm });
                break;


            case 0x2c ... 0x2d: // sub: Zb/Ib, Zv/Iz (Z -> a)
                uops.push_back({ uop_sub,
                    (u16)(setOpSize(opsz) | set_cond | extflag | use_ra | use_imm),
                    { to_ureg(reg64_a), 0, 0, to_ureg(reg64_a) },
                    imm });
                break;


            case 0x30 ... 0x33: // xor: Eb/Gb, Ev/Gv, Gb/Eb, Gv/Ev
                uops.push_back({ uop_xor,
                    (u16)(setOpSize(opsz) | set_cond | extflag | use_ra | use_rb | use_rc),
                    { to_ureg(opdst, !temp_op), to_ureg(opsrc), to_ureg(load_reg, load_reg), to_ureg(opdst) },
                    imm });
                break;


            case 0x34 ... 0x35: // xor: Zb/Ib, Zv/Iz (Z -> a)
                uops.push_back({ uop_xor,
                    (u16)(setOpSize(opsz) | set_cond | extflag | use_ra | use_imm),
                    { to_ureg(reg64_a), 0, 0, to_ureg(reg64_a) },
                    imm });
                break;


            case 0x38 ... 0x3b: // cmp: Eb/Gb, Ev/Gv, Gb/Eb, Gv/Ev
                uops.push_back({ uop_sub, // cmp = sub without writing destination
                    (u16)(setOpSize(opsz) | set_cond | extflag | use_ra | use_rb | use_rc),
                    { to_ureg(opdst, !temp_op), to_ureg(opsrc), to_ureg(load_reg, load_reg), 0 },
                    imm });
                break;


            case 0x3c ... 0x3d: // cmp: Zb/Ib, Zv/Iz (Z -> a)
                uops.push_back({ uop_sub,
                    (u16)(setOpSize(opsz) | set_cond | extflag | use_ra | use_imm),
                    { to_ureg(reg64_a), 0, 0, 0 },
                    imm });
                break;


            case 0x50 ... 0x57: // push r64/r16
            {   // register in opcode extension
                u8 src = ((op.meta.has_rex ? ((op.bytes[op.meta.off_rex] & rex::b) << 3) : 0) | (opcode & 0b0111));
                uops.push_back({ uop_push,
                    (u16)(setOpSize(opsz) | use_ra | use_rb),
                    { to_ureg(reg64_sp), to_ureg(src), 0, to_ureg(reg64_sp) },
                    imm });
                break;
            }


            case 0x58 ... 0x5f: // pop r64/r16
            {
                u8 dest = ((op.meta.has_rex ? ((op.bytes[op.meta.off_rex] & rex::b) << 3) : 0) | (opcode & 0b0111));
                uops.push_back({ uop_pop,
                    (u16)(setOpSize(opsz) | use_ra | rc_dest ),
                    { to_ureg(reg64_sp), 0, to_ureg(reg64_sp), to_ureg(dest) },
                    imm });
                break;
            }


            case 0x63: // movsxd: Gv/Ev
                // pass source width in imm, then sx(reg, srcw, opsz)
                uops.push_back({ uop_move,
                    // always extend, resize as usual
                    // alternative: extend only with opsz 8
                    (u16)(setOpSize(opsz) | extflag | rd_extend | use_ra | use_rb),
                    { to_ureg(opdst, !!(opsz & 0b11)), to_ureg(load_reg ? load_reg : opsrc), 0, to_ureg(opdst) },
                    (imm | (opsz == 2 ? 2 : 4))});
                break;


            case 0x68: // push: Iz
            case 0x6a: // push: Ib
                uops.push_back({ uop_push,
                    (u16)(setOpSize(opsz) | use_ra | use_imm),
                    { to_ureg(reg64_sp), 0, 0, to_ureg(reg64_sp) },
                    imm });
                break;


            case 0x69: // imul: Gv/Ev/Iz
            case 0x6b: // imul: Gv/Ev/Ib
                uops.push_back({ uop_imul,
                    (u16)(setOpSize(opsz) | set_cond | extflag | use_ra ),
                    { to_ureg(load_reg ? load_reg : opsrc), 0, 0, to_ureg(opdst) },
                    imm });
                break;


            case 0x70 ... 0x7f: // jcc short: Ib
                // condition code in lower nibble, uops will be similar
                uops.push_back({ (u16)(uop_brancho + (opcode & 0xf)),
                    (u16)(setOpSize(opsz) | use_cond | use_imm),
                    { 0, 0, 0, 0 },
                    // this immediate is only extended to opsz!
                    // need to sign extend again to addr width in uop
                    imm });
                break;


            case 0x80 ... 0x81: // immediate groups 1 Eb/Ib, Ev/Iz
            case 0x83: // Ev/Ib
            {
                u16 uopcode = 0;
                switch(modrm::get_reg(modrm))
                {   
                    case 0b000: uopcode = uop_add; break;
                    case 0b001: uopcode = uop_or;  break;
                    case 0b010: uopcode = uop_adc; break;
                    case 0b011: uopcode = uop_sbb; break;
                    case 0b100: uopcode = uop_and; break;
                    case 0b101: uopcode = uop_sub; break;
                    case 0b110: uopcode = uop_xor; break;
                    case 0b111: uopcode = uop_sub; break;
                }
                uops.push_back({ uopcode,
                    (u16)(setOpSize(opsz) | set_cond | extflag | use_ra | use_rc | use_imm),
                    { to_ureg(opdst, !temp_op), 0, to_ureg(load_reg, load_reg),
                        to_ureg(opdst, (modrm::get_reg(modrm) != 0b111)) },
                    imm });
                break;
            }


            case 0x84 ... 0x85: // test: Eb/Gb, Ev/Gv
                uops.push_back({ uop_and, // and without writing destination
                    (u16)(setOpSize(opsz) | set_cond | extflag | use_ra | use_rb | use_rc),
                    { to_ureg(opdst, !temp_op), to_ureg(opsrc), to_ureg(load_reg, load_reg), 0 },
                    imm });
                break;


            case 0x86 ... 0x87: // xchg: Eb/Gb, Ev/Gv
                uops.push_back({ uop_xchg,
                    (u16)(setOpSize(opsz) | extflag | use_ra | use_rb | rc_dest ),
                    { to_ureg(opdst), to_ureg(opsrc), to_ureg(opsrc), to_ureg(opdst) },
                    imm });

                if(temp_op) uops.push_back({ uop_move,
                    (u16)(setOpSize(opsz) | extflag | use_ra ),
                    { to_ureg(load_reg), 0, 0, to_ureg(opsrc) },
                    imm });
                break;


            case 0x88 ... 0x8c: // mov: Eb/Gb, Ev/Gv, Gb/Eb, Gv/Ev
                // memory operand is destination, no load needed
                if(load_reg && temp_op)
                    uops.clear();
                
                // mem -> reg, modify implicit load
                if(load_reg && !temp_op)
                    uops.back().regs[3] = to_ureg(opdst);

                // reg -> reg
                else if(!temp_op) uops.push_back({ uop_move,
                    (u16)(setOpSize(opsz) | (opsz != 4 ? rd_resize : 0) | use_ra | use_rb),
                    // dependence for byte/word dest
                    { to_ureg(opdst, !!(opsz & 0b11)), to_ureg(load_reg ? load_reg : opsrc), 0, to_ureg(opdst) },
                    imm });

                // reg -> mem, use the implicit store
                else opdst = opsrc;
                break;


            case 0x8d: // lea: Gv/Mv
                uops.clear(); // don't need to load from address
                uops.push_back({ uop_lea,
                    (u16)(setOpSize(ldsz) | use_ra | use_rb | use_rc | use_imm),
                    { to_ureg(rexb_ex, (sib_useb | (!op.off_sib && modrm::get_rm(modrm) != 0b101))),
                        to_ureg(sib_idx, sib_usei), segbase, to_ureg(load_reg) },
                    (((u64)sib_scl << 32) | (bitmask(32) & (i32)displ)) });
                break;


            case 0x8e: // mov: Sw, Ew (fs = reg4, gs = reg5)
                mod_reg &= 0b111; // rex.r ignored for segments
                opdst = (mod_reg == 4) ? reg64_fsbase : (mod_reg == 5) ? reg64_gsbase : 0;
                
                // invalid segment
                if(!opdst)
                    raise_ud();

                // direct load
                else if(load_reg)
                    uops.back().regs[3] = to_ureg(opdst);
                else uops.push_back({ uop_move,
                    (u16)(setOpSize(opsz) | use_rb),
                    { 0, to_ureg(opsrc), 0, to_ureg(opdst) },
                    imm });
                break;


            case 0x8f: // group 1a
                switch(modrm::get_reg(modrm))
                {
                    case 0b000: // pop: Ef
                        // either pop or load+store

                        // no memory op, normal pop
                        if(!load_reg) uops.push_back({ uop_pop,
                            (u16)(setOpSize(opsz) | use_ra | rc_dest ),
                            { to_ureg(reg64_sp), 0, to_ureg(reg64_sp), to_ureg(opdst) },
                            imm });

                        // pop into temporary, store temporary to target
                        else
                        {
                            uops.clear(); // no load!
                            uops.push_back({ uop_pop,
                                (u16)(setOpSize(opsz) | use_ra | rc_dest ),
                                { to_ureg(reg64_sp), 0, to_ureg(reg64_sp), to_ureg(load_reg) }, // reuse allocd load dest
                                imm });
                            opdst = load_reg; // use implicit store
                        }
                        break;
                    default:
                        raise_ud();
                        break;
                }
                break;


            case 0x90 ... 0x97: // xchg: Rv, Rv (a <-> r)
            {   // 10010rrr (+ rex.b)
                u8 dest = ((op.meta.has_rex ? ((op.bytes[op.meta.off_rex] & rex::b) << 3) : 0) | (opcode & 0b0111));

                if(!dest) // xchg a, a -> nop
                    uops.push_back({ uop_nop,
                        (use_imm),
                        { 0, 0, 0, 0 },
                        0x90 });
                else // actual xchg operation
                {
                    uops.push_back({ uop_xchg,
                        (u16)(setOpSize(opsz) | extflag | use_ra | use_rb | rc_dest),
                        { to_ureg(reg64_a), to_ureg(dest), to_ureg(dest), to_ureg(reg64_a) },
                        imm });
                    
                    if(temp_op) uops.push_back({ uop_move,
                        (u16)(setOpSize(opsz) | extflag | use_ra ),
                        { to_ureg(load_reg), 0, 0, to_ureg(opsrc) },
                        imm });
                }
                break;
            }


            case 0x98: // cbw/cwde/cdqe: a -> a
            case 0x99: // cwd/cdq/cqo a -> d:a
                TODO;
                break;

            case 0x9c: // pushf
                uops.push_back({ uop_pushx,
                    (u16)(setOpSize(opsz) | use_cond | use_ra | use_imm),
                    { to_ureg(reg64_sp), 0, 0, to_ureg(reg64_sp) },
                    px_flags });
                break;

            case 0x9d: // popf
                uops.push_back({ uop_popx,
                    (u16)(setOpSize(opsz) | set_cond | use_ra | rc_dest | use_imm),
                    { to_ureg(reg64_sp), 0, to_ureg(reg64_sp), 0 },
                    px_flags });
                break;


            case 0xa0 ... 0xa1: // moffset mov: Zb/Ob, Zv/Ov
                // (moffset is located in imm, not displacement!)
                // load reg_a from vaddr64
                uops.push_back({ uop_ld64,
                    (u16)(setOpSize(opsz) | use_ra | use_imm),
                    { 0, 0, 0, to_ureg(reg64_a) },
                    imm });
                break;


            case 0xa2 ... 0xa3: // moffset mov: Ob/Zb, Ov/Zv
                // store reg_a to vaddr64
                uops.push_back({ uop_st,
                    (u16)(setOpSize(opsz) | use_rb | use_imm),
                    { 0, to_ureg(reg64_a), 0, 0 },
                    imm });
                break;


            case 0xa8 ... 0xa9: // test: Zb/Ib, Zv/Iv
                uops.push_back({ uop_and,
                    (u16)(setOpSize(opsz) | set_cond | extflag | use_ra | use_imm ),
                    { to_ureg(reg64_a), 0, 0, 0 },
                    imm });
                break;


            case 0xb0 ... 0xb7: // mov: immediate b -> r8
            {   // 10110rrr (+ rex.b)
                u8 dest = ((op.meta.has_rex ? ((op.bytes[op.meta.off_rex] & rex::b) << 3) : 0)
                    | (opcode & 0b0111));
            
                // use set uop, /rb is dependent on old register value
                uops.push_back({ uop_set,
                    (use_imm | use_ra),
                    { to_ureg(dest), 0, 0, to_ureg(dest) },
                    imm });
                break;
            }


            case 0xb8 ... 0xbf: // mov: immediate w/d/q -> r
            {   // 10111rrr (+ rex.b)
                u8 dest = ((op.meta.has_rex ? ((op.bytes[op.meta.off_rex] & rex::b) << 3) : 0) | (opcode & 0b0111));

                // use set uop, r16 is dependent on old register value, r32/r64 are not
                uops.push_back({ uop_set,
                    (u16)(setOpSize(opsz) | use_imm | (opsz == 2 ? use_ra : 0)),
                    { to_ureg(dest), 0, 0, to_ureg(dest) },
                    imm });
                break;
            }


            case 0xc0: // shift group 2 Eb/Ib
            case 0xc1: // shift group 2 Ev/Ib
            // counts are always masked to 5/6 bits, this will be done inside the uop
            {
                u16 uopcode = 0;
                switch(modrm::get_reg(modrm))
                {   // TODO don't always set flags!
                    case 0b000: uopcode = uop_rol; break;
                    case 0b001: uopcode = uop_ror; break;
                    case 0b010: uopcode = uop_rcl; break;
                    case 0b011: uopcode = uop_rcr; break;
                    case 0b100: uopcode = uop_lsl; break;
                    case 0b101: uopcode = uop_rsl; break;
                    case 0b110: raise_ud();        break; // no sal
                    case 0b111: uopcode = uop_rsa; break;
                }
                if(!uopcode) [[unlikely]] break;

                uops.push_back({ uopcode,
                    (u16)(setOpSize(opsz) | set_cond | extflag | use_ra | use_imm),
                    { to_ureg(load_reg ? load_reg : opdst), 0, 0, to_ureg(opdst) },
                    imm });
                break;
            }


            case 0xc2: // ret Iw (pop imm bytes)
            case 0xc3: // ret
                // we can't just pop into rip -> pop tmp; branch tmp
                load_reg = get_tmpreg(regs_gp);
                uops.push_back({ uop_pop,
                    (u16)(setOpSize(8) | use_ra | rc_dest ),
                    { to_ureg(reg64_sp), 0, to_ureg(reg64_sp), to_ureg(load_reg) },
                    imm });
                uops.push_back({ uop_branch,
                    (u16)(setOpSize(8) | use_ra | use_imm),
                    { to_ureg(load_reg), 0, 0, 0 },
                    imm });
                break;


            case 0xc6: // mov group 11
            case 0xc7:
                switch(modrm::get_reg(modrm))
                {
                    case 0b000: // mov: Eb/Ib, Ev/Iz
                        uops.clear(); // don't load

                        // reg target
                        if(!load_reg) uops.push_back({ uop_set,
                            (u16)(setOpSize(opsz) | extflag | use_imm | use_ra),
                            { to_ureg(opdst), 0, 0, to_ureg(opdst) },
                            imm });

                        // mem target
                        else storeimm = 1; // use implicit lea+store 
                        break;
                    default:
                        raise_ud();
                }
                break;


            case 0xc8: // enter Iw/Ib
                TODO;
                break;


            case 0xc9: // leave
                TODO;
                break;


            case 0xca: // far ret Iw
            case 0xcb: // far ret
                // same as near ret
                load_reg = get_tmpreg(regs_gp);
                uops.push_back({ uop_pop,
                    (u16)(setOpSize(8) | use_ra | rc_dest ),
                    { to_ureg(reg64_sp), 0, to_ureg(reg64_sp), to_ureg(load_reg) },
                    imm });
                uops.push_back({ uop_branch,
                    (u16)(setOpSize(8) | use_ra | use_imm),
                    { to_ureg(load_reg), 0, 0, 0 },
                    imm });
                break;


            case 0xcc: // int3
                uops.push_back({ uop_int,
                    use_imm,
                    {0},
                    ex_BP });
                break;


            case 0xcd: // int Ib
                uops.push_back({ uop_int,
                    use_imm,
                    {0},
                    imm });
                break;


            case 0xcf: // iret
                TODO;
                break;


            case 0xd0: // shift group 2 Eb/1
            case 0xd1: // shift group 2 Ev/1
            {
                u16 uopcode = 0;
                switch(modrm::get_reg(modrm))
                {   // TODO don't always set flags!
                    case 0b000: uopcode = uop_rol; break;
                    case 0b001: uopcode = uop_ror; break;
                    case 0b010: uopcode = uop_rcl; break;
                    case 0b011: uopcode = uop_rcr; break;
                    case 0b100: uopcode = uop_lsl; break;
                    case 0b101: uopcode = uop_rsl; break;
                    case 0b110: raise_ud();        break; // no sal
                    case 0b111: uopcode = uop_rsa; break;
                }
                if(!uopcode) [[unlikely]] break;

                uops.push_back({ uopcode,
                    (u16)(setOpSize(opsz) | set_cond | extflag | use_ra | use_imm),
                    { to_ureg(load_reg ? load_reg : opdst), 0, 0, to_ureg(opdst) },
                    (imm | 0x1) });
                break;
            }


            case 0xd2: // shift group 2 Eb/Zb (Z -> cl)
            case 0xd3: // shift group 2 Ev/Zb (Z -> cl)
            {
                u16 uopcode = 0;
                switch(modrm::get_reg(modrm))
                {   // TODO don't always set flags!
                    case 0b000: uopcode = uop_rol; break;
                    case 0b001: uopcode = uop_ror; break;
                    case 0b010: uopcode = uop_rcl; break;
                    case 0b011: uopcode = uop_rcr; break;
                    case 0b100: uopcode = uop_lsl; break;
                    case 0b101: uopcode = uop_rsl; break;
                    case 0b110: raise_ud();        break; // no sal
                    case 0b111: uopcode = uop_rsa; break;
                }
                if(!uopcode) [[unlikely]] break;

                uops.push_back({ uopcode,
                    (u16)(setOpSize(opsz) | set_cond | extflag | use_ra | use_rb),
                    { to_ureg(load_reg ? load_reg : opdst), to_ureg(reg64_c), 0, to_ureg(opdst) },
                    imm });
                break;
            }


            case 0xe3: // jrcxz: Jb/Rv
                // jump if reg_c is zero
                uops.push_back({ uop_branchrz,
                    (u16)(setOpSize(opsz) | use_ra | use_imm),
                    { to_ureg(reg64_c), 0, 0, 0 },
                    imm });
                break;


            case 0xe8: // call: Jz
                // call needs push relative + branch
                uops.push_back({ uop_pushx,
                    (u16)(setOpSize(opsz) | use_ra | use_imm),
                    { to_ureg(reg64_sp), 0, 0, to_ureg(reg64_sp) },
                    px_rip });
                [[fallthrough]];

            case 0xe9: // jmp: Jz
            case 0xeb: // jmp: Jb
                uops.push_back({ uop_branchr,
                    (u16)(setOpSize(opsz) | use_imm),
                    { 0, 0, 0, 0 },
                    imm });
                break;


            case 0xf1: // int1
                // TODO; map x64 to core exceptions
                uops.push_back({ uop_int,
                    use_imm,
                    {0},
                    to_core_except(ex64_DB) });
                break;


            case 0xf4: // halt
                uops.push_back({ uop_int,
                    use_imm,
                    {0},
                    /*ex_HALT*/ ex_GP }); // we don't have ring 0 yet
                break;


            case 0xf5: // cmc
                uops.push_back({ uop_cmc,
                    (use_cond | set_cond),
                    {0},
                    0});
                break;


            case 0xf6: // unary group 3
                switch(modrm::get_reg(modrm))
                {
                    case 0b000: // test: Eb/Ib
                        uops.push_back({ uop_and,
                            (u16)(setOpSize(opsz) | set_cond | extflag | use_ra | use_imm ),
                            { to_ureg(load_reg ? load_reg : opdst), 0, 0, 0 },
                            imm });
                        break;

                    case 0b001: // undefined
                        raise_ud();
                        break;

                    case 0b010: // not: Eb
                        uops.push_back({ uop_not,
                            (u16)(setOpSize(opsz) | extflag | use_ra),
                            { to_ureg(load_reg ? load_reg : opdst), 0, 0, to_ureg(opdst) },
                            imm });
                        break;

                    case 0b011: // neg: Eb
                        uops.push_back({ uop_neg,
                            (u16)(setOpSize(opsz) | set_cond | extflag | use_ra),
                            { to_ureg(load_reg ? load_reg : opdst), 0, 0, to_ureg(opdst) },
                            imm });
                        break;

                    case 0b100: // mul: Eb
                        uops.push_back({ uop_mul,
                            (u16)(setOpSize(opsz) | extflag | use_ra | use_rb | rc_dest ),
                            { to_ureg(reg64_a), to_ureg(load_reg ? load_reg : opdst), to_ureg(reg64_d), to_ureg(reg64_a) },
                            imm });
                        break;

                    case 0b101: // imul: Eb
                        uops.push_back({ uop_imul,
                            (u16)(setOpSize(opsz) | set_cond | extflag | use_ra | use_rb | rc_dest ),
                            { to_ureg(reg64_a), to_ureg(load_reg ? load_reg : opdst), to_ureg(reg64_d), to_ureg(reg64_a) },
                            imm });
                        break;

                    case 0b110: // div: Eb
                        temp_op = 0;
                        uops.push_back({ uop_div8,
                            (u16)(setOpSize(opsz) | extflag | use_ra | use_rb),
                            { to_ureg(reg64_a), to_ureg(load_reg ? load_reg : opdst), 0, to_ureg(reg64_a) },
                            imm });
                        break;

                    case 0b111: // idiv: Eb
                        temp_op = 0;
                        uops.push_back({ uop_idiv8,
                            (u16)(setOpSize(opsz) | extflag | use_ra | use_rb),
                            { to_ureg(reg64_a), to_ureg(load_reg ? load_reg : opdst), 0, to_ureg(reg64_a) },
                            imm });
                        break;
                        break;
                }
                break;


            case 0xf7: // unary group 3
                switch(modrm::get_reg(modrm))
                {
                    case 0b000: // test: Ev/Iz
                        uops.push_back({ uop_and,
                            (u16)(setOpSize(opsz) | set_cond | extflag | use_ra | use_imm ),
                            { to_ureg(load_reg ? load_reg : opdst), 0, 0, 0 },
                            imm });
                        break;

                    case 0b001: // undefined
                        raise_ud();
                        break;

                    case 0b010: // not: Ev
                        uops.push_back({ uop_not,
                            (u16)(setOpSize(opsz) | extflag | use_ra),
                            { to_ureg(load_reg ? load_reg : opdst), 0, 0, to_ureg(opdst) },
                            imm });
                        break;

                    case 0b011: // neg: Ev
                        uops.push_back({ uop_neg,
                            (u16)(setOpSize(opsz) | set_cond | extflag | use_ra),
                            { to_ureg(load_reg ? load_reg : opdst), 0, 0, to_ureg(opdst) },
                            imm });
                        break;

                    // octaword mul, 4 operands are enough:
                    // imul a,s -> d,a
                    case 0b100: // mul: Ev
                        uops.push_back({ uop_mul,
                            (u16)(setOpSize(opsz) | extflag | use_ra | use_rb | rc_dest ),
                            { to_ureg(reg64_a), to_ureg(load_reg ? load_reg : opdst), to_ureg(reg64_d), to_ureg(reg64_a) },
                            imm });
                        break;

                    // same as mul above
                    case 0b101: // imul: Ev
                        uops.push_back({ uop_imul,
                            (u16)(setOpSize(opsz) | set_cond | extflag | use_ra | use_rb | rc_dest ),
                            { to_ureg(reg64_a), to_ureg(load_reg ? load_reg : opdst), to_ureg(reg64_d), to_ureg(reg64_a) },
                            imm });
                        break;


                    // div needs 3 input, 2 output
                    // solution: split up into quotient, remainder and copy uop to respect dependences
                    // divq: d,a,s -> tempQ; divr -> d,a,s -> tempR; mov tempQ,tempR -> a,d;
                    case 0b110: // div: Ev
                    {
                        u8 tempq = get_tmpreg(regs_gp);
                        u8 tempr = get_tmpreg(regs_gp);
                        temp_op = 0; // disable store

                        uops.push_back({ uop_divq,
                            (u16)(setOpSize(opsz) | extflag | use_ra | use_rb | use_rc ),
                            { to_ureg(reg64_a), to_ureg(reg64_d), to_ureg(load_reg ? load_reg : opdst), to_ureg(tempq) },
                            imm });
                        uops.push_back({ uop_divr,
                            (u16)(setOpSize(opsz) | extflag | use_ra | use_rb | use_rc ),
                            { to_ureg(reg64_a), to_ureg(reg64_d), to_ureg(load_reg ? load_reg : opdst), to_ureg(tempr) },
                            imm });
                        uops.push_back({ uop_copy2,
                            (u16)(setOpSize(opsz) | use_ra | use_rb | rc_dest ),
                            { to_ureg(tempq), to_ureg(tempr), to_ureg(reg64_a), to_ureg(reg64_d) },
                            imm });
                        break;
                    }

                    case 0b111: // idiv: Ev
                    {
                        u8 tempq = get_tmpreg(regs_gp);
                        u8 tempr = get_tmpreg(regs_gp);
                        temp_op = 0;

                        uops.push_back({ uop_idivq,
                            (u16)(setOpSize(opsz) | extflag | use_ra | use_rb | use_rc ),
                            { to_ureg(reg64_a), to_ureg(reg64_d), to_ureg(load_reg ? load_reg : opdst), to_ureg(tempq) },
                            imm });
                        uops.push_back({ uop_idivr,
                            (u16)(setOpSize(opsz) | extflag | use_ra | use_rb | use_rc ),
                            { to_ureg(reg64_a), to_ureg(reg64_d), to_ureg(load_reg ? load_reg : opdst), to_ureg(tempr) },
                            imm });
                        uops.push_back({ uop_copy2,
                            (u16)(setOpSize(opsz) | use_ra | use_rb | rc_dest ),
                            { to_ureg(tempq), to_ureg(tempr), to_ureg(reg64_a), to_ureg(reg64_d) },
                            imm });
                        break;
                    }
                }
                break;


            case 0xf8 ... 0xf9: // clc, stc
            case 0xfc ... 0xfd: // cld, std
                uops.push_back({ (u16)(uop_clc + (opcode & 0x7)),
                    (use_cond | set_cond),
                    { 0 },
                    0 });
                break;

            
            case 0xfa ... 0xfb: // cli, sti
                uops.push_back({ uop_int,
                    use_imm,
                    { 0 },
                    ex_GP }); // no iopl
                break;


            // inc/dec 4
            case 0xfe:
                switch(modrm::get_reg(modrm))
                {
                    default:
                        raise_ud();
                        break;

                    case 0b000: // inc
                        uops.push_back({ uop_add,
                            (u16)(setOpSize(opsz) | set_cond | extflag | use_ra | use_rc | use_imm),
                            { to_ureg(opdst, !temp_op), 0, to_ureg(load_reg, load_reg),
                                to_ureg(opdst, (modrm::get_reg(modrm) != 0b111)) },
                            (imm | 1) });
                        break;

                    case 0b001: // dec
                        uops.push_back({ uop_sub,
                            (u16)(setOpSize(opsz) | set_cond | extflag | use_ra | use_rc | use_imm),
                            { to_ureg(opdst, !temp_op), 0, to_ureg(load_reg, load_reg),
                                to_ureg(opdst, (modrm::get_reg(modrm) != 0b111)) },
                            (imm | 1) });
                        break;
                }
                break;


            // inc/dec 5
            case 0xff:
                switch(modrm::get_reg(modrm))
                {
                    default:
                        raise_ud();
                        break;

                    case 0b000: // inc
                        uops.push_back({ uop_add,
                            (u16)(setOpSize(opsz) | set_cond | extflag | use_ra | use_rc | use_imm),
                            { to_ureg(opdst, !temp_op), 0, to_ureg(load_reg, load_reg),
                                to_ureg(opdst, (modrm::get_reg(modrm) != 0b111)) },
                            1 });
                        break;

                    case 0b001: // dec
                        uops.push_back({ uop_sub,
                            (u16)(setOpSize(opsz) | set_cond | extflag | use_ra | use_rc | use_imm),
                            { to_ureg(opdst, !temp_op), 0, to_ureg(load_reg, load_reg),
                                to_ureg(opdst, (modrm::get_reg(modrm) != 0b111)) },
                            1 });
                        break;

                    case 0b010: // call Ev
                        uops.push_back({ uop_pushx,
                            (u16)(setOpSize(8) | use_ra | use_imm),
                            { to_ureg(reg64_sp), 0, 0, to_ureg(reg64_sp) },
                            px_rip });
                        uops.push_back({ uop_branch,
                            (u16)(setOpSize(opsz) | use_ra),
                            { to_ureg(load_reg ? load_reg : opdst), 0, 0, 0 },
                            imm });
                        break;

                    case 0b011: // call + seg
                        TODO;
                        break;

                    case 0b100: // jmp Ev
                        uops.push_back({ uop_branch,
                            (u16)(setOpSize(opsz) | use_ra),
                            { to_ureg(load_reg ? load_reg : opdst), 0, 0, 0 },
                            imm });
                        break;

                    case 0b101: // jmp + seg
                        TODO;
                        break;
                }
                break;
        }
    }
    else if(op.meta.op_mode == 1) // two byte opcodes
    {
        switch(opcode)
        {
            default:
                raise_ud();
                break;

            
            case 0x05: // syscall
            case 0x07: // sysret
            case 0x08: // invd
            case 0x09: // wbinvd
            case 0x0d: // prefetchw
                raise_ud();
                break;


            case 0x0a: // ud2
                raise_ud();
                break;


            case 0x19: // nop
            case 0x1c: // nop
            case 0x1d: // nop
            case 0x1e: // nop
            case 0x1f: // nop
                uops.push_back({ uop_nop,
                    (use_imm),
                    { 0, 0, 0, 0 },
                    opcode });
                break;


            case 0x31: // rdtsc
                uops.push_back({ uop_rdtsc,
                    (rc_dest),
                    { 0, 0, to_ureg(reg64_d), to_ureg(reg64_a) },
                    0 });
                break;


            case 0x40 ... 0x4f: // cmovcc
                // cmov will load, then move depending on condition
                uops.push_back({ (u16)(uop_movo + (opcode & 0xf)),
                    (u16)(setOpSize(opsz) | (opsz != 4 ? rd_resize : 0) | use_cond | use_ra | use_rb),
                    // dependence for word dest
                    { to_ureg(opdst, (opsz == 2)), to_ureg(load_reg ? load_reg : opsrc), 0, to_ureg(opdst) },
                    imm });
                break;


            case 0x80 ... 0x8f: // jcc long
                uops.push_back({ (u16)(uop_brancho + (opcode & 0xf)),
                    (u16)(setOpSize(opsz) | use_cond | use_imm),
                    { 0, 0, 0, 0 },
                    imm });
                break;

            
            case 0x90 ... 0x9f: // setcc Eb
                // this is not the uop 'setcc'!
                TODO;
                break;


            case 0xa0: // pushfs
                uops.push_back({ uop_push,
                    (u16)(setOpSize(opsz) | use_ra | use_rb),
                    { to_ureg(reg64_sp), to_ureg(reg64_fsbase), 0, to_ureg(reg64_sp) },
                    0 });
                break;


            case 0xa1: // popfs
                uops.push_back({ uop_pop,
                    (u16)(setOpSize(opsz) | use_ra | rc_dest ),
                    { to_ureg(reg64_sp), 0, to_ureg(reg64_sp), to_ureg(reg64_fsbase) },
                    imm });
                break;


            case 0xa2: // cpuid
                TODO;
                break;


            case 0xa3: // bt
                TODO;
                break;


            case 0xa4 ... 0xa5: // shld
                TODO;
                break;


            case 0xa8: // pushgs
                uops.push_back({ uop_push,
                    (u16)(setOpSize(opsz) | use_ra | use_rb),
                    { to_ureg(reg64_sp), to_ureg(reg64_gsbase), 0, to_ureg(reg64_sp) },
                    0 });
                break;


            case 0xa9: // popgs
                uops.push_back({ uop_pop,
                    (u16)(setOpSize(opsz) | use_ra | rc_dest ),
                    { to_ureg(reg64_sp), 0, to_ureg(reg64_sp), to_ureg(reg64_gsbase) },
                    imm });
                break;


            case 0xab: // bts
                TODO;
                break;


            case 0xac ... 0xad: // shrd
                TODO;
                break;


            case 0xaf: // imul Gv/Ev
                uops.push_back({ uop_imul,
                    (u16)(setOpSize(opsz) | set_cond | extflag | use_ra | use_rb ),
                    { to_ureg(opdst), to_ureg(load_reg ? load_reg : opsrc), 0, to_ureg(opdst) },
                    0 });
                break;


            case 0xb0 ... 0xb1: // cmpxchg
                TODO;
                break;


            case 0xb3: // btr
                TODO;
                break;


            case 0xb6 ... 0xb7: // movzx
                TODO;
                break;


            case 0xb8:
                if(op.meta.has_g1 == 0xf3) // popcnt
                    TODO;
                else
                    raise_ud();
                break;


            case 0xb9: // ud1
                raise_ud();
                break;


            case 0xba:
                switch(modrm::get_reg(modrm))
                {
                    default:
                        raise_ud();
                        break;

                    case 0b100: // bt
                        break;

                    case 0b101: // bts
                        break;

                    case 0b110: // btr
                        break;

                    case 0b111: // btc
                        break;
                }
                break;


            case 0xbb: // btc
                TODO;
                break;


            case 0xbc: // bsf / tzcnt
                TODO;
                break;


            case 0xbd: // bsr / lzcnt
                TODO;
                break;


            case 0xbe ... 0xbf: // movsx
                uops.push_back({ uop_move,
                    (u16)(setOpSize(opsz) | extflag | rd_extend | use_ra | use_rb),
                    { to_ureg(opdst, (opsz == 2)), to_ureg(load_reg ? load_reg : opsrc), 0, to_ureg(opdst) },
                    (imm | (opcode == 0xbe ? 1 : 2))});
                break;


            case 0xc0 ... 0xc1: // xadd
                TODO;
                break;


            case 0xc8 ... 0xcf: // bswap
                TODO;
                break;


            case 0xff: // ud0
                raise_ud();
                break;
        }
    }
    // else if(op.meta.op_mode == 2) // three byte opcodes
    // {

    // }
    else [[unlikely]] // missed something?
        raise_ud();

    // finalize gp instructions
    if(is_gp(op))
    {
        // if the target is a memory location, we need to finish the macro op with a store
        // - recalculate the address, this brings minimum uops for load/compute/store to 4
        // -- (lea/lda with 2 outputs may 'solve' this, but then we lose a source address operand..)
        // - then store
        // if required, store can be disabled by resetting temp_op (e.g. div)
        if(temp_op && !operands.empty() && op.off_modrm && (modrm::get_mod(modrm) != 0b11) &&
           x64def::is_rmop(operands[0]))
        {
            addr_reg = get_tmpreg(regs_gp);

            // operands see implied lda
            uops.push_back({ uop_lea,
                // TODO
                (u16)(setOpSize(adsz) | use_ra | use_rb | use_rc | use_imm),
                { to_ureg(rexb_ex, (sib_useb | (!op.off_sib && modrm::get_rm(modrm) != 0b101))),
                    to_ureg(sib_idx, sib_usei), segbase, to_ureg(addr_reg) },
                (((u64)sib_scl << 32) | (bitmask(32) & (i32)displ)) });

            uops.push_back({ uop_st,
                (u16)(setOpSize(ldsz) | use_ra | use_rb),
                { to_ureg(addr_reg), to_ureg(opdst, !storeimm), 0, 0 },
                imm });
        }
    }

    // set macro positions
    if(!uops.empty())
    {
        uops.front().control |= mop_first;
        uops.back().control  |= mop_last;
    }

    util::log(LOG_64_PIPE1, "        Uop bundle:", (uops.empty() ? " EMPTY" : ""));

    // add collected uops to the queue
    for(uop op : uops)
    {
        if(op.imm) op.control |= use_imm; // adjusted register

        util::log(LOG_64_PIPE1, "          ", uop_readable(op).str());
        uqueue->push_back(state.cycle + DECODE_LATENCY, op);
    }

    util::log(LOG_64_PIPE1, "");

    // 0: all uops have been inserted
    // 1: some uops are still missing (e.g. MSROM?) TBD
    return 0;
}