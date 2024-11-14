// o3 RISC simulator
// 
// out-of-order core
// - main functions
// - decode
// - rename/alloc
// - issue
// - execute/memory
// - commit
//
// Lukas Heine 2021

#include "core.hh"
#include "cconf.hh"

#include "uops.hh"

Core::Core(LatchQueue<uop>* uqueue, Simulator::SimulatorState& state, MemoryManager& mmu,
    Frontend& fe) : uqueue(uqueue), state(state), mmu(mmu), fe(fe)
{
    RenameTable trt = { 
        {0}, {0}, {0}, // forward lookup (allocated)
        {0}, {0}, {0}, // forward lookup (commited)
        {0}, {0}, {0}, // reverse lookup
        std::deque<u8>(), std::deque<u8>(), std::deque<u8>(), std::deque<u8>(), std::deque<u8>(),
    };
    rrt = trt;

    // init free lists, all physical registers (except r0) are available
    for(u16 i = 1; i < REGCLS_0_RNREG; i++) rrt.gp_freelist.push_back(i);
    for(u16 i = 1; i < REGCLS_1_RNREG; i++) rrt.fp_freelist.push_back(i);
    for(u16 i = 1; i < REGCLS_2_RNREG; i++) rrt.vr_freelist.push_back(i);
    for(u16 i = 1; i < CCREG_CNT;      i++) rrt.cc_freelist.push_back(i);

    id_ra = new LatchQueue<uop>(ID_RA_SIZE + DECODE_WIDTH);
    rob   = new LatchQueue<ROBEntry>(ROB_SIZE + ALLOC_WIDTH);
    ldq   = new LatchQueue<ROBEntry*>(LQUEUE_SIZE + ALLOC_WIDTH);

    next_inactive = 0;

    util::log(LOG_CORE_INIT, "Core initialized with:");
    util::log(LOG_CORE_INIT, "        Decode width: ", dec_u<0>, DECODE_WIDTH);
    util::log(LOG_CORE_INIT, "        Alloc  width: ", dec_u<0>, ALLOC_WIDTH);
    util::log(LOG_CORE_INIT, "        Issue  width: ", dec_u<0>, ISSUE_WIDTH);
    util::log(LOG_CORE_INIT, "        Commit width: ", dec_u<0>, COMMIT_WIDTH);
    util::log(LOG_CORE_INIT, "");
}

Core::~Core()
{
    delete state.arf;

    delete id_ra;
    delete rob;
    delete ldq;
}

// one complete backend cycle
u32 Core::cycle()
{
    mmu.refresh();

    if(!(state.active & core_active))
    {
        util::log(LOG_CORE_ARF, "\nCore inactive.");
        return 1;
    }

    // print rs directly after alloc, not here
    // util::log(LOG_STATE_PRE, "Functional Units:\n", rs);
    util::log(LOG_STATE_PRE, "ROB:\n", rob_readable(8).str());

    decode();
    alloc();
    issue();
    execute();
    commit();

    // util::log(LOG_STATE_POST, "Functional Units:\n", rs);
    util::log(LOG_STATE_POST, "ROB:\n", rob_readable(8).str());

    util::log(LOG_CORE_PRF, "\nPRF GP:\n", prf_readable(0).str());
    util::log(LOG_CORE_PRF, "PRF CC:\n",   prf_readable(3).str());
    util::log(LOG_CORE_ARF, "ARF GP:\n", state.arf_readable(0).str());

    return 0;
}

// clear latches, iterators and statuses
u8 Core::flush()
{
    std::deque<u8>* cur_freelist = nullptr;

    // free all allocated physical registers
    for(u16 i = 0; i < rob->size(); i++)
    {
        switch(getOpPrefix(rob->at(UINT64_MAX, i).op))
        {
            default:
            case 0x0: // control
            case 0x1: // alu
                cur_freelist = &rrt.gp_freelist;
                break;
            case 0x2: // fpu
                cur_freelist = &rrt.fp_freelist;
                break;
            case 0x3: // vec int
            case 0x4: // vec float
                cur_freelist = &rrt.vr_freelist;
                break;
        }
        // todo make a commit free list instead of searching ROB
        if(rob->at(UINT64_MAX, i).op.regs[r_rd])
            cur_freelist->push_back(rob->at(UINT64_MAX, i).op.regs[r_rd]);
    }

    // reset rename tables to commited state
    // this might be a problem when resetting late? idk needs to be tested, might need to copy register contents
    std::memcpy(rrt.gp, rrt.gc, REGCLS_0_CNT);
    std::memcpy(rrt.fp, rrt.fc, REGCLS_0_CNT);
    std::memcpy(rrt.vr, rrt.vc, REGCLS_0_CNT);


    // or reset to last commited condition?
    rrt.cc_freelist.clear();
    for(u16 i = 1; i < CCREG_CNT; i++) rrt.cc_freelist.push_back(i);

    // clear latches
    uqueue->clear();
    id_ra->clear();
    rob->clear();
    ldq->clear();

    // reset instruction trace
    state.in_flight.erase((state.in_flight.begin() + 1), state.in_flight.end());
    state.seq_addrs.clear();
    state.refetch_at = 0;
    seq_at_alloc     = 0;
    next_inactive    = 0;

    // clear rs
    for(auto& rsp : rs.ports)
    {
        rsp.busy = 0;
        for(auto& fu : rsp.fus)
        {
            fu.busy  = 0;
            fu.cycle = 0;
            fu.re    = nullptr;
        }
    }

    fe.flush();

    state.flushes++;

    return 0;
}

// 'decode' DECODE_WIDTH uops from uQ into pipeline latch
// > check #UD and look up opcode mnemonics
u32 Core::decode()
{
    if(!id_ra->ready(state.cycle)) // latency condition not met
    {
        util::log(LOG_CORE_PIPE1, "ID__:   Decode busy.\n");
        return 1;
    }

    if(next_inactive & id_active && !(state.active & fe_active)) state.active &= ~id_active;
    if( !(state.active & id_active) )
    {
        util::log(LOG_CORE_PIPE1, "ID__:   Decode inactive.\n");
        return 1;
    }

    uop          cur_op;
    uopinfo      cur_info;
    u16*         cur_ctrl;

    // iterate decode slots
    for(u32 slot = 0; slot < DECODE_WIDTH; slot++)
    {
        if(id_ra->size() >= ID_RA_SIZE + DECODE_WIDTH)
        {
            util::log(LOG_CORE_PIPE1, "ID__: * ID/RA latch is full. Not decoding any instructions.");
            break;
        }

        if(!uqueue->ready(state.cycle))
        {   // instructions not available yet
            util::log(LOG_CORE_PIPE1, "ID__: * uQueue content is not ready. Not decoding more instructions.");
            break;
        }

        try
        {   
            cur_op   =  uqueue->get_front(state.cycle);
            cur_info =  uopmap.at(cur_op.opcode);
            cur_ctrl =  &cur_op.control;

            // invalid control bits set
            if( (*cur_ctrl | cur_info.ctrl_mask) != cur_info.ctrl_mask )
            { // always do this? shouldn't change anything
                util::log(LOG_CORE_PIPE2, "ID.", dec_u<0>, slot, ": * Invalid control bits detected, bits merged with mask.");
                *cur_ctrl &= cur_info.ctrl_mask;
            }

            // rc is either source or destination
            if(*cur_ctrl & rc_dest) *cur_ctrl &= ~use_rc;

            // clear unused operands and bits, this is needed since 0 is not always neutral element (e.g. uop_and)
            // if zero reg is needed as source, this should be modified (for now we don't really need that..)
            for(u8 i = 0; i < ((*cur_ctrl & rc_dest) ? 2 : 3); i++)
            {
                if(!(*cur_ctrl & (use_ra << i))) cur_op.regs[i] = 0;
                if(cur_op.regs[i] == 0) *cur_ctrl &= ~(use_ra << i);
            }

            if(!(*cur_ctrl & use_imm)) cur_op.imm = 0;

            // check register bounds
            // this needs some work for conversion uops
            u16 arf_sz = getARFSize(cur_op);
            for(u8 i = 0; i < 4; i++)
                if(cur_op.regs[i] >= arf_sz)
                {
                    util::log(LOG_CORE_PIPE1, "ID.", dec_u<0>, slot, ": * Invalid register reference r", +cur_op.regs[i],
                        ". Injecting #REF.");
                    cur_op.opcode       = uop_int;
                    *(u32*) cur_op.regs = 0;
                    cur_op.imm          = ex_REG;
                }

            util::log(LOG_CORE_PIPE1, "ID.", dec_u<0>, slot, ":   Decoded instruction ", cur_op, " to: ");
            util::log(LOG_CORE_PIPE1, "          ", uop_readable(cur_op).str());
        }
        catch(const LatchStallException& ls)
        {   // instructions not available yet
            util::log(LOG_CORE_PIPE1, "ID__: * uQueue ", ls.what(), " Not decoding more instructions.");
            break;
        }
        catch(const LatchEmptyException& le)
        {   // uq empty
            util::log(LOG_CORE_PIPE1, "ID__: * uQueue ", le.what(), " Not decoding more instructions.");
            // no more instructions incoming
            if( !(state.active & fe_active) ) next_inactive |= id_active;
            break;
        }
        catch(const std::out_of_range& ud)
        {   // invalid opcode, replace instruction with int #UD to guarantee precise exception
            util::log(LOG_CORE_PIPE1, "ID.", dec_u<0>, slot, ": * Undefined opcode ", hex_u<16>, cur_op.opcode, ". Injecting #UD.");
            cur_op.opcode       = uop_int;   // interrupt
            cur_op.control      = use_imm;
            *(u32*) cur_op.regs = 0;        // null registers
            cur_op.imm          = ex_UD;    // #UD
        }

        // no need to catch exceptions here since size is already checked
        id_ra->push_back((state.cycle + DECODE_LATENCY), cur_op);
    }

    util::log(5, "");
    return 1;
}

// rename registers and allocate/fill ROB entries
u32 Core::alloc()
{
    if(next_inactive & ra_active) state.active &= ~ra_active;
    if( !(state.active & ra_active) )
    {
        util::log(LOG_CORE_PIPE1, "RA__:   Rename/alloc inactive.\n");
        return 1;
    }

    if(!rob->ready(state.cycle)) // latency condition not met
    {
        util::log(LOG_CORE_PIPE1, "RA__:   Rename/allocate busy.\n");
        return 1;
    }

    for(u32 slot = 0; slot < ALLOC_WIDTH; slot++)
    {
        if(rob->size() >= ROB_SIZE + ALLOC_WIDTH)
        {
            util::log(LOG_CORE_PIPE1, "RA__: * No available ROB slots. Not allocating RRT/ROB entries.");
            break;
        }

        if(!id_ra->ready(state.cycle))
        {   // instructions not available yet
            util::log(LOG_CORE_PIPE1, "RA__: * ID/RA latch is not ready. Not decoding more instructions.");
            break;
        }

        // first check RRT for free slots
        // then get uop from latch and allocate
        try
        {
            uop*            cur_op_peek  = &id_ra->front(state.cycle);
            u8              cur_opclass  = getOpPrefix(*cur_op_peek);
            std::deque<u8>* cur_freelist = nullptr;
            u8*             cur_rrt      = nullptr;
            u8*             cur_trr      = nullptr;
            void*           cur_prf      = nullptr; 
            void*           cur_arf      = nullptr;
            size_t          cur_regsz    = 0;

            switch(cur_opclass)
            {   // modify to match target register classes
                default:
                case 0x0: // control
                case 0x1: // alu
                    cur_freelist = &rrt.gp_freelist;
                    cur_rrt   = rrt.gp;
                    cur_trr   = rrt.pg;
                    cur_prf   = prf.gp;
                    cur_arf   = state.arf->gp;
                    cur_regsz = REGCLS_0_SIZE;
                    break;
                case 0x2: // fpu
                    cur_freelist = &rrt.fp_freelist;
                    cur_rrt   = rrt.fp;
                    cur_trr   = rrt.pf;
                    cur_prf   = prf.fp;
                    cur_arf   = state.arf->fp;
                    cur_regsz = REGCLS_1_SIZE;
                    break;
                case 0x3: // vec int
                case 0x4: // vec float
                    cur_freelist = &rrt.vr_freelist;
                    cur_rrt   = rrt.vr;
                    cur_trr   = rrt.rv;
                    cur_prf   = prf.vr;
                    cur_arf   = state.arf->vr;
                    cur_regsz = REGCLS_2_SIZE;
                    break;
            }

            // try to get op from latch, allocate free RRT slot, transform uop and place into ROB
            util::log(LOG_CORE_PIPE1, "RA.", dec_u<0>, slot, ":   Got ", *cur_op_peek, " from latch.");

            // check if we need to load source registers
            // otherwise we can not initialize registers from outside the core
            u8 loadcount = 0;
            for(u8 sreg = 0; sreg < 3; sreg++)
                if(cur_op_peek->regs[sreg] && (cur_op_peek->control & (use_ra << sreg)) &&
                   !cur_rrt[cur_op_peek->regs[sreg]])
                    loadcount++;

            if(cur_freelist->size() < (loadcount + (cur_op_peek->control & rc_dest) ? 2 : 1))
            {
                util::log(LOG_CORE_PIPE1, "RA.", dec_u<0>, slot, ": * Not enough physical registers from register class available.");
                break;
            }

            if((cur_op_peek->control & set_cond) && rrt.cc_freelist.empty())
            {
                util::log(LOG_CORE_PIPE1, "RA.", dec_u<0>, slot, ": * No condition register available.");
                break;
            }

            if((cur_op_peek->control & use_cond) && (rrt.cc_freelist.size() == CCREG_CNT - 1))
            {
                util::log(LOG_CORE_PIPE1, "RA.", dec_u<0>, slot, ": * No condition register was set.");
                cur_op_peek->control &= ~use_cond; // discard condition dependence
            }

            if(is_load(*cur_op_peek) && ldq->size() >= LQUEUE_SIZE + ALLOC_WIDTH)
            {
                util::log(LOG_CORE_PIPE1, "RA.", dec_u<0>, slot, ": * LoadQ is full. Pipeline stalled.");
                break;
            }

            // resources available, take uop from latch
            uop cur_op = id_ra->get_front(state.cycle);

            // actual renaming
            // - rename sources according to forward rrt to carry over true dependences
            // - rename destination, forward rrt will contain last phreg modifying archreg

            // physical register tag, don't rename r0! also need to check for possible second dst
            u8 rc = (cur_op.control & rc_dest) ? cur_op.regs[r_rc] : 0;
            u8 rd = cur_op.regs[r_rd];

            u8 ccu = (cur_op.control & use_cond) ? rrt.cc_lastused.back() : 0;
            if(ccu) util::log(LOG_CORE_PIPE2, "RA.", dec_u<0>, slot, ":     Condition register ", dec_u<0>, +ccu, " used.");
            u8 ccs = (cur_op.control & set_cond) ? rrt.cc_freelist.front() : 0;
            if(ccs)
            {
                rrt.cc_lastused.push_back(ccs);
                rrt.cc_freelist.pop_front();
                util::log(LOG_CORE_PIPE2, "RA.", dec_u<0>, slot, ":     Condition register ", dec_u<0>, +ccs, " set.");
            }
            u8 phregc = (rc ? cur_freelist->front() : 0);
            if(rc) cur_freelist->pop_front();
            u8 phregd = (rd ? cur_freelist->front() : 0);
            if(rd) cur_freelist->pop_front();

            // check source registers
            for(u8 sreg = 0; sreg < 3; sreg++)
                // not r0 and register is actually used
                if(cur_op.regs[sreg] && (cur_op.control & (use_ra << sreg)))
                {
                    if(cur_rrt[cur_op.regs[sreg]])
                    { // found a match
                        util::log(LOG_CORE_PIPE2, "RA.", dec_u<0>, slot, ":     Src r", dec_u<0>, +cur_op.regs[sreg],
                            " is mapped to p", +cur_rrt[cur_op.regs[sreg]], ".");
                        cur_op.regs[sreg] = cur_rrt[cur_op.regs[sreg]];
                    }
                    else
                    {
                        util::log(LOG_CORE_PIPE2, "RA.", dec_u<0>, slot, ": *   Src r", dec_u<0>, +cur_op.regs[sreg],
                            " not mapped yet, fetching from ARF.");

                        // create a new mapping, we checked for free regs already
                        u8 loadreg = cur_freelist->front(); 
                        cur_rrt[cur_op.regs[sreg]] = loadreg;
                        cur_trr[loadreg] = cur_op.regs[sreg];

                        // copy contents from the respective ARF register
                        std::memcpy((void*)((uptr)cur_prf + cur_regsz * loadreg),
                            (void*)((uptr)cur_arf + cur_regsz * cur_op.regs[sreg]), cur_regsz);
                        
                        util::log(LOG_CORE_PIPE2, "RA.", dec_u<0>, slot, ":     r", dec_u<0>, +cur_op.regs[sreg],
                            " renamed to p", dec_u<0>, +loadreg, ".");
                        cur_op.regs[sreg] = loadreg;
                        cur_freelist->pop_front();
                    }
                }

            if(rc)
            {
                cur_rrt[rc] = phregc;
                cur_trr[phregc] = rc;
                util::log(LOG_CORE_PIPE2, "RA.", dec_u<0>, slot, ":     Dst r", dec_u<0>, +cur_op.regs[2], " renamed to p", dec_u<0>,
                     +phregc, ".");
                cur_op.regs[r_rc] = phregc;
            }

            // store renamed register to table
            if(rd)
            {
                cur_rrt[rd] = phregd;
                cur_trr[phregd] = rd;
                util::log(LOG_CORE_PIPE2, "RA.", dec_u<0>, slot, ":     Dst r", dec_u<0>, +cur_op.regs[3], " renamed to p", dec_u<0>,
                    +phregd, ".");
                cur_op.regs[r_rd] = phregd;
            }
            
            MM::MemoryRef mref = MM::zero_mref;

            if(is_branch(cur_op))
                mref.mode  = MM::mr_branch;

            // always add the sequential RIP here in case we need it for rip-relative calculations (only disp32)
            mref.vaddr = state.seq_addrs.empty() ? 0 : state.seq_addrs.at(seq_at_alloc);

            // advance RIP indices
            if(cur_op.control & mop_last)
            {
                seq_at_alloc++;
            }

            ROBEntry re = { mref, cur_op, commit_unavail, exec_waiting, ex_NONE, ccu, ccs};
            rob->push_back((state.cycle + ALLOC_LATENCY), re);
            
            util::log(LOG_CORE_PIPE1, "RA.", dec_u<0>, slot,":   Sent ", cur_op, " to ROB.");

            // allocate entry in load queue
            if(is_load(cur_op))
            {
                // uop exec is too late to modify this, so we have to do it here
                u8 delay = (cur_op.control & imm_delay) ? getImmDelay(cur_op) : 1;

                ldq->push_back((state.cycle + delay), &rob->back());
                util::log(LOG_CORE_PIPE2, "RA.", dec_u<0>, slot,":   Allocated LoadQ entry. Additional delay ",
                    dec_u<0>, delay-1, ".");
            }
        }
        catch(const LatchStallException& ls)
        {   // instructions not available yet
            util::log(LOG_CORE_PIPE1, "RA__: * Input latch ", ls.what(), " No allocation done.");
            break;
        }
        catch(const LatchEmptyException& le)
        {   // uq empty
            util::log(LOG_CORE_PIPE1, "RA__: * Input latch ", le.what(), " No allocation done.");
            if( !(state.active & id_active) ) next_inactive |= ra_active;
            break;
        }
    }

    util::log(LOG_CORE_PIPE1, "");
    return 1;
}

// issue uops from ROB to available RS ports
// one port can issue to one attached FU each cycle
u32 Core::issue()
{
    // shut down backend since the ROB is empty and not expecting more uops
    if(next_inactive & is_active) state.active &= ~is_active & ~ex_active & ~co_active;
    if( !(state.active & is_active) )
    {
        util::log(LOG_CORE_PIPE1, "IS__:   Issue inactive.\n");
        return 1;
    }

    u16 check_next = 0; // ROBEntry index
    u8  issued = 0;

    for(auto& rsp : rs.ports)
        if(rsp.busy) rsp.busy--; // decrease lock

    // - check unissued uops (re.in_exec)
    // - determine possible ports/FUs using mask
    // - check source dependences for previous not commited uops
    // -- register is not found -> issue (should be mapped)
    // -- register is found but not ready -> defer uop
    // -- register is found and ready -> issue (is mapped)
    // - check condition dependences
    // - issue to available port and disable it for this cycle (plus latency)

    for(u32 slot = 0; slot < ISSUE_WIDTH; slot++)
    {
        ROBEntry* cur_re   = nullptr; // scanned ROBEntry
        RSPort* issue_port = nullptr; // issue ready port
        FUInfo* issue_fu   = nullptr; // issue ready fu
        uop*    cur_op     = nullptr; // uop in ROBEntry

        u8 cur_op_ports    = 0; // possible ports
        u8 cur_op_fu       = 0; // fu type

        // try to find available FU at specified ports
        // TODO FU will finish this cycle
        auto find_fu = [&] {
            for(auto& rsp : get_rsports(cur_op_ports))
                if(!rsp->busy)
                    for(auto& fu : rsp->fus) // all FUs at available matching port
                        if(!fu.cycle && ((cur_op_fu == fu_any) || (fu.type == cur_op_fu)))
                        {
                            issue_port  = rsp;
                            issue_fu    = &fu;
                            return; // break from lambda after match
                        }
        };

        // scan dependences up to ROB index
        auto unavail_dependences = [&](const u16 limit) {
            u8 sources[3] = { cur_op->regs[r_ra], cur_op->regs[r_rb], cur_op->regs[r_rc] };
            u8 cur_op_cls = getOpClassId(*cur_op);
            
            u16 re_i;
            for(re_i = 0; re_i < limit; re_i++)
            {
                ROBEntry* re = &rob->at(state.cycle, re_i);
                if(cur_op_cls != getOpClassId(re->op)) continue; // different regfile from uop
                for(u8 r = 0; r < 3; r++)
                    if(sources[r] && (cur_op->control & (use_ra<<r)) &&
                        (re->c_ready == commit_unavail) && ((re->op.regs[r_rd] == sources[r]) ||
                            ((re->op.control & rc_dest) && (re->op.regs[r_rc] == sources[r]))))
                    {   // register not r0, used, present, not ready
                        util::log(LOG_CORE_PIPE3, "IS.", dec_u<0>, slot, ":     Source p", dec_u<3>, +sources[r],
                            " not ready at ROB index ", re_i, ".");
                        return 1;
                    }
                    else if((re->c_ready == commit_unavail) &&
                        ((cur_re->op.control & use_cond) && (cur_re->cc_use == re->cc_set)))
                    {   // condition used, not ready
                        util::log(LOG_CORE_PIPE3, "IS.", dec_u<0>, slot, ":     Condition reg c", dec_u<2>, +cur_re->cc_use,
                            " not ready at ROB index ", re_i, ".");
                        return 1;
                    }
            }
            return 0;
        };

        try
        {
            util::log(LOG_CORE_PIPE2, "IS.", dec_u<0>, slot, ":   Checking uops from RE ", check_next, ".");
            u16 i = 0;
            for(i = check_next; i <= rob->size() && i <= ISSUE_DEPTH; i++)
            {
                check_next = i + 1; // don't check this uop again in another issue slot
                cur_re     = &rob->at(state.cycle, i);
                if((cur_re->in_exec == exec_waiting) && (cur_re->c_ready == commit_unavail))
                {
                    cur_op       = &cur_re->op;
                    cur_op_ports = uopmap.at(cur_op->opcode).ports;
                    cur_op_fu    = uopmap.at(cur_op->opcode).fu_type;

                    find_fu();
                    if(unavail_dependences(i)) continue; // not ready, find another uop
                    break;
                }
            }

            util::log(LOG_CORE_PIPE1, "IS.", dec_u<0>, slot, ":   Trying to issue uop ", *cur_op);

            // we checked all issue slots
            if(i == ISSUE_DEPTH)
            {
                util::log(LOG_CORE_PIPE1, "IS.", dec_u<0>, slot, ": * Scheduler entries exhausted.");
                break; // no uop in range can be issued
            }

            if(!issue_port && !issue_fu) // no matching FU at any port
            {
                util::log(LOG_CORE_PIPE1, "IS.", dec_u<0>, slot, ": * No matching FU or port available.");
                continue;
            }
            else if(!issue_port) // no matching port
            {
                util::log(LOG_CORE_PIPE1, "IS.", dec_u<0>, slot, ": * No RS port available.");
                continue;
            }

            if(!issue_fu) // no matching FU, this is only a fallback
            {
                util::log(LOG_CORE_PIPE1, "IS.", dec_u<0>, slot, ": * No FU of needed type available.");
                continue;
            }
            else util::log(LOG_CORE_PIPE1, "IS.", dec_u<0>, slot, ":   Ready FU found at port ", +issue_port->id, ":", +issue_fu->id,
                ", uop issued.");

            // FU and dependences are now checked, issue uops
            issue_port->busy = ISSUE_LATENCY ? ISSUE_LATENCY : 1; // enforce RS port latency
            issue_fu->cycle  = state.cycle + ISSUE_LATENCY;       // earliest cycle for execution start
            issue_fu->re     = cur_re;
            cur_re->in_exec  = exec_running;
            issued++;
        }
        catch(const std::out_of_range& oor)
        {   // all in execution or unmet dependences
            util::log(LOG_CORE_PIPE1, "IS.", dec_u<0>, slot, ": * No uops can be issued.");
            break;
        }
        catch(const LatchStallException& ls)
        {   // uops not available yet
            util::log(LOG_CORE_PIPE1, "IS.", dec_u<0>, slot, ": * ROB ", ls.what(), " No uops issued.");
            break;
        }
        catch(const LatchEmptyException& le)
        {
            util::log(LOG_CORE_PIPE1, "IS.", dec_u<0>, slot, ": * ROB ", le.what(), " No uops issued.");
            if( !(state.active & ra_active) ) next_inactive |= is_active;
            break;
        }
    }

    util::log(LOG_CORE_BUF, "");
    util::log(LOG_CORE_BUF, "Functional Units:\n", rs);

    if(issued) util::log(LOG_CORE_PIPE1, "IS__:   ", dec_u<0>, +issued, " uop(s) issued this cycle.");

    util::log(LOG_CORE_PIPE1, "");
    return 0;
}

// execute uops on all allocated FUs
u32 Core::execute()
{
    if(next_inactive & ex_active) state.active &= ~ex_active;
    if( !(state.active & ex_active) )
    {
        util::log(LOG_CORE_PIPE1, "EX__:   Execute inactive.\n");
        return 1;
    }

    if(rob->empty() && !(state.active & is_active)) 
    {
        util::log(LOG_CORE_PIPE1, "EX__:   ROB is empty. No uops can be executed.\n");
        next_inactive |= ex_active;
        return 0;
    }

    // execute load requests
    // find incomplete load
    // execute it
    // set exception if needed

    ROBEntry* re        = nullptr;
    MM::MemoryRef* mref = nullptr;

    mmu.refresh();

    for(u32 slot = 0; slot < LOAD_WIDTH; slot++)
    {
        try
        {
            for(u32 i = 0; i < ldq->size(); i++)
            {
                if(ldq->at(state.cycle, i)->mref.ready == MM::mr_exready)
                {
                    util::log(LOG_CORE_PIPE2, "LD.", dec_u<0>, slot, ":   Ready loadQ entry found.");

                    re = ldq->at(state.cycle, i);
                    mref = &re->mref;

                    // set commit ready from mmu?
                    MM::MemoryRequest mreq = { mref, &re->except };
                    mmu.get(mreq, MM::p_r);
                    break;
                }
                else if(ldq->at(state.cycle, i)->mref.ready == MM::mr_valready && !ldq->at(state.cycle, i)->c_ready)
                    ldq->at(state.cycle, i)->c_ready = state.cycle;
            }
        }
        catch(const MemoryManagerException& me)
        {
            util::log(0, me.what());
            re->except = setExcept(ex_PF, expf_present | (state.ring == pl_user ? expf_user : 0));
        }
        catch(const LatchException& le) { /* util::log(0, le.what()); */ }
    }

    // start execution when fu.busy cycle is reached
    for(auto& port : rs.ports)
        for(auto& fu : port.fus)
        {
            if(fu.cycle == state.cycle) fu.busy = uopmap.at(fu.re->op.opcode).latency;
            if(fu.busy)
            {
                util::log(LOG_CORE_BUF, "EX__:   Port ", +port.id, ":", +fu.id, " (", str_w<6>,
                    fu_type_str[fu.type], ") in execution. ", +fu.busy, " cycle(s) left.");
                
                if(fu.busy == 1) // last cycle before latency, actually execute uop
                {
                    if(is_cvt(fu.re->op)) [[unlikely]]
                    {
                        TODO;
                        auto regcls = getCvtClassIds(fu.re->op);
                        switch(classid_pair(regcls.first, regcls.second))
                        {
                            default: // same regclasses, shouldn't use convert here
                                break;
                            case classid_pair(regs_gp, regs_fp):
                                run_cvt<REGCLS_0_SIZE, REGCLS_1_SIZE>(*fu.re, prf.gp, prf.fp);
                                break;
                            case classid_pair(regs_fp, regs_gp):
                                run_cvt<REGCLS_1_SIZE, REGCLS_0_SIZE>(*fu.re, prf.fp, prf.gp);
                                break;
                            // ...
                        }
                    }
                    else switch(getOpClassId(fu.re->op))
                    {
                        default:
                        case regs_gp: run_uop<REGCLS_0_SIZE>(*fu.re, prf.gp); break;
                        case regs_fp: run_uop<REGCLS_1_SIZE>(*fu.re, prf.fp); break;
                        case regs_vr: run_uop<REGCLS_2_SIZE>(*fu.re, prf.vr); break;
                    }
                    fu.re = nullptr;
                    fu.cycle = 0;
                }
                fu.busy--;
            }
            else util::log(LOG_CORE_BUF, "EX__:   Port ", +port.id, ":", +fu.id , " (", str_w<6>,
                fu_type_str[fu.type], ") not in execution.");
        }

    util::log(LOG_CORE_PIPE1, "");
    return 0;
}

// commit core state to arf in order
u32 Core::commit()
{
    if(next_inactive & co_active) state.active &= ~co_active;
    if( !(state.active & co_active) )
    {
        util::log(LOG_CORE_PIPE1, "CO__:   Commit inactive.\n");
        return 1;
    }

    // todo track macro op latencies!
    // how?
    // - extend uop format (x) or
    // - hash vector of uop.opcodes and look that up
    // - something else
    
    ROBEntry        cur_re;
    uop*            cur_op       = nullptr;
    std::deque<u8>* cur_freelist = nullptr;
    u8*             cur_rrt      = nullptr; // allocd
    u8*             cur_rct      = nullptr; // commited
    u8*             cur_trr      = nullptr; // reverse
    void*           cur_pregc    = nullptr; // raw pointer to register content
    void*           cur_aregc    = nullptr;
    void*           cur_pregd    = nullptr;
    void*           cur_aregd    = nullptr;
    size_t          cur_regsz    = 0;

    // util::log(LOG_CORE_BUF, "ROB:\n", rob_readable(8).str());

    for(u32 slot = 0; slot < COMMIT_WIDTH; slot++)
    {
        try
        {   
            // something requested a refetch starting from this instruction pointer
            // we don't really care what the exact uop is since it might be modified by SMC or garbage data
            if(state.refetch_active && state.refetch_at == state.in_flight.front())
            {
                util::log(LOG_CORE_PIPE3, "CO.", dec_u<0>, slot, ":   Refetch instruction pointer reached.");

                fe.set_fetchaddr(state.refetch_at);
                flush();
                state.active = fe_active | core_active;
                state.refetch_active = 0;
                break;
            }
           
            // todo only commit entire macro ops (?)
            // if uop is FOM, peek uops until EOM is found, if any are not c_ready, fast return and stall

            // check if this uop can be commited in the current cycle
            u64 ready = rob->front(state.cycle).c_ready;
            if(!!ready == commit_ready && (ready <= state.cycle))
            {
                cur_re = rob->get_front(state.cycle);
                cur_op = &cur_re.op;

                // exception occured at ROB head, print status and shut down (exceptions can not be handled yet)
                if(cur_re.except)
                {
                    util::log(LOG_CORE_PIPE1, "CO.", dec_u<0>, slot, ":   Exception detected.");

                    if(FAST_EXCEPT)
                    {   // todo might refetch if IFETCH and USER is set? (-> TBD when ring 0 is implemented)
                        util::log(LOG_CORE_PIPE1, "CO.", dec_u<0>, slot, ":   Exception ", getExceptNum(cur_re.except),
                            " ", exception_str[getExceptNum(cur_re.except)], ". Error code ", hex_u<16>,
                            getExceptEC(cur_re.except), "\n");
                        state.active = 0;
                        state.commited_micro++;
                        state.exception = cur_re.except;
                        return 1;
                    }
                    else TODO;
                    break;
                }


                // look up arf and metadata from rename table
                switch(getOpPrefix(*cur_op))
                {
                    default:
                    case 0x0: // control
                    case 0x1: // alu
                        cur_freelist = &rrt.gp_freelist;
                        cur_rrt      = rrt.gp;
                        cur_rct      = rrt.gc;
                        cur_trr      = rrt.pg;
                        cur_pregc    = &prf.gp[cur_op->regs[r_rc]];
                        cur_aregc    = &state.arf->gp[cur_trr[cur_op->regs[r_rc]]];
                        cur_pregd    = &prf.gp[cur_op->regs[r_rd]];
                        cur_aregd    = &state.arf->gp[cur_trr[cur_op->regs[r_rd]]];
                        cur_regsz    = REGCLS_0_SIZE;
                        break;
                    case 0x2: // fpu
                        cur_freelist = &rrt.fp_freelist;
                        cur_rrt      = rrt.fp;
                        cur_rct      = rrt.fc;
                        cur_trr      = rrt.pf;
                        cur_pregc    = &prf.fp[cur_op->regs[r_rc]];
                        cur_aregc    = &state.arf->fp[cur_trr[cur_op->regs[r_rc]]];
                        cur_pregd    = &prf.fp[cur_op->regs[r_rd]];
                        cur_aregd    = &state.arf->fp[cur_trr[cur_op->regs[r_rd]]];
                        cur_regsz    = REGCLS_1_SIZE;
                        break;
                    case 0x3: // vec int
                    case 0x4: // vec float
                        cur_freelist = &rrt.vr_freelist;
                        cur_rrt      = rrt.vr;
                        cur_rct      = rrt.vc;
                        cur_trr      = rrt.rv;
                        cur_pregc    = &prf.vr[cur_op->regs[r_rc]];
                        cur_aregc    = &state.arf->vr[cur_trr[cur_op->regs[r_rc]]];
                        cur_pregd    = &prf.vr[cur_op->regs[r_rd]];
                        cur_aregd    = &state.arf->vr[cur_trr[cur_op->regs[r_rd]]];
                        cur_regsz    = REGCLS_2_SIZE;
                        break;
                }

                // actual commit to ARF (ignore invalid loads, those will be handled later)
                if(!(is_load(*cur_op) && cur_re.mref.mode == MM::mr_invalid)) [[likely]]
                {
                    // todo partial register writes
                    if(cur_op->control & rc_dest)
                        std::memcpy(cur_aregc, cur_pregc, cur_regsz);
                    std::memcpy(cur_aregd, cur_pregd, cur_regsz);
                    util::log(LOG_CORE_PIPE1, "CO.", dec_u<0>, slot, ":   ARF updated.");
                }


                if(cur_op->control & rc_dest)
                    util::log(LOG_CORE_PIPE1, "          p", dec_u<0>, +cur_op->regs[r_rc], " -> r", +cur_trr[cur_op->regs[r_rc]]);

                util::log(LOG_CORE_PIPE1, "          p", dec_u<0>, +cur_op->regs[r_rd], " -> r", +cur_trr[cur_op->regs[r_rd]]);

                // remove preg mappings
                if((cur_op->control & rc_dest) && cur_op->regs[r_rc])
                {
                    // update commit table
                    cur_rct[cur_trr[cur_op->regs[r_rc]]] = cur_op->regs[r_rc];

                    // free physical (destination) register
                    cur_freelist->push_back(cur_op->regs[r_rc]);
                    cur_trr[cur_op->regs[r_rc]] = 0;

                    // unmap physical register from rename table if this is the last write
                    if(cur_rrt[cur_trr[cur_op->regs[r_rc]]] == cur_op->regs[r_rc])
                        cur_rrt[cur_trr[cur_op->regs[r_rc]]] = 0;
                }

                if(cur_op->regs[r_rd])
                {
                    // update commit table
                    cur_rct[cur_trr[cur_op->regs[r_rd]]] = cur_op->regs[r_rd];

                    // free physical (destination) register
                    cur_freelist->push_back(cur_op->regs[r_rd]);
                    cur_trr[cur_op->regs[r_rd]] = 0;

                    // unmap physical register from rename table if this is the last write
                    if(cur_rrt[cur_trr[cur_op->regs[r_rd]]] == cur_op->regs[r_rd])
                        cur_rrt[cur_trr[cur_op->regs[r_rd]]] = 0;
                }

                util::log(LOG_CORE_PIPE3, "CO.", dec_u<0>, slot, ":   Registers deallocated.");

                // free condition register as soon as a different one is commited
                if(cur_re.cc_set && (cur_re.cc_set != rrt.cc_lastused.front()))
                {                    
                    rrt.cc_freelist.push_back(rrt.cc_lastused.front());
                    rrt.cc_lastused.pop_front();

                    std::memcpy(&state.arf->cc, &prf.cc[rrt.cc_lastused.front()], CCREG_SIZE);
                    util::log(LOG_CORE_PIPE3, "CO.", dec_u<0>, slot, ":   Condition register ", dec_u<0>,
                        +rrt.cc_lastused.front(), " committed.");
                }

                // handle load
                // this can probably be moved above the unmapping process since flush will do it anyway
                if(is_load(*cur_op))
                {
                    util::log(LOG_CORE_PIPE1, "CO.", dec_u<0>, slot, ":   Load detected.");
                    // misspeculated? address was written to since load executed

                    // probably easiest to just flush instead of depth searching ROB dependences
                    if(ldq->front(state.cycle)->mref.mode == MM::mr_invalid)
                    {
                        // load was misspeculated
                        util::log(LOG_CORE_PIPE2, "CO.", dec_u<0>, slot, ":   Load is invalid.");
                        fe.set_fetchaddr(state.in_flight.front());
                        flush();
                        state.active = fe_active | core_active; // restart frontend
                        ldq->clear();
                        break; // this load is not allowed to commit!!
                    }

                    // should never throw since loads can only enter ROB with a matching LQ entry
                    ldq->pop_front();
                }

                // handle store
                if(is_store(*cur_op))
                {
                    util::log(LOG_CORE_PIPE1, "CO.", dec_u<0>, slot, ":   Store detected.");

                    for(u64 i = 0; i < ldq->size(); i++)
                        try
                        {
                            auto& lref = ldq->at(state.cycle, i)->mref;
                            // store address was used to load wrong value
                            // invalidate and then refetch when load tries to commit
                            if(lref.ready && mmu.is_alias(lref.vaddr, lref.size, cur_re.mref.vaddr, cur_re.mref.size))
                            {
                                util::log(LOG_CORE_PIPE2, "CO.", dec_u<0>, slot,
                                    ":     Misspeculated load found. LoadQ entry invalidated.");
                                lref.mode = MM::mr_invalid;
                            }
                        }
                        catch(const LatchException& me) { /* don't care */ }

                    MM::MemoryRequest mreq = { &cur_re.mref, &cur_re.except, 0 };
                    mmu.put(mreq);

                    if(cur_re.except)
                    {   // store raised an exception, this *will* commit next
                        flush();
                        rob->push_front((state.cycle + 0), { MM::zero_mref, { uop_int, 0, {0}, cur_re.except },
                            state.cycle, cur_re.except, exec_running, 0, 0 });
                        continue;
                    }

                    // we need to flush the pipeline when a store to in flight uop address is detected
                    // flush, then refetch when uop tries to commit (fetch will wait for store request to complete) 

                    // do we write to a in flight instruction?
                    for(u64 i = 0; i < state.seq_addrs.size(); i++)
                    {
                        if(mmu.is_alias(cur_re.mref.vaddr, cur_re.mref.size,
                            state.in_flight.at(i), (state.seq_addrs.at(i) - state.in_flight.at(i))))
                        {
                            state.refetch_at = state.in_flight.at(i);
                            state.refetch_active = 1;
                            util::log(LOG_CORE_PIPE2, "CO.", dec_u<0>, slot, "SMC at v.", hex_u<64>, state.refetch_at,
                                " detected. Target and following instructions will be refetched.");
                            break;
                        }
                    }
                }

                // handle branches
                if(is_branch(*cur_op))
                {
                    util::log(LOG_CORE_PIPE1, "CO.", dec_u<0>, slot, ":   Branch detected. Sequential instruction at v.", hex_u<64>,
                        state.seq_addrs.front());

                    u64 nextrip = 0;

                    // relative or absolute jump
                    // - target in memref.vaddr
                    // - memref.size == -1 if not taken

                    if(cur_re.mref.mode == MM::mr_branch)
                    {
                        nextrip = cur_re.mref.vaddr;
                        util::log(LOG_CORE_PIPE2, "           Next instruction at v.", hex_u<64>, nextrip);
                    }
                    else throw SimulatorException();

                    util::log(LOG_CORE_PIPE3, "             Memory reference: ", cur_re.mref);

                    // not taken
                    // if we also pass the current rip, we can do the branch update inside the uop
                    if(cur_re.mref.size == UINT64_MAX)
                    {
                        fe.bp->update(state.in_flight.front(), nextrip, 0);
                        nextrip = state.seq_addrs.front();
                    }
                    else fe.bp->update(state.in_flight.front(), nextrip, 1);

                    // mispredicted, refetch
                    // this will never throw, there will **always** be two elements in in_flight at this point 
                    if(state.in_flight.at(1) != nextrip)
                    {
                        fe.set_fetchaddr(nextrip);
                        flush();
                        state.in_flight.push_back(nextrip);
                        state.active = fe_active | core_active; // restart frontend
                    }
                }

                // only update instruction pointers when last in bundle commits
                if(cur_op->control & mop_last)
                {
                    state.in_flight.pop_front();

                    // this will be empty after flush
                    if(state.seq_addrs.size())
                    {
                        state.seq_addrs.pop_front();
                        seq_at_alloc--;
                    }

                    // set visible ip to next pending instruction
                    state.arf->ip.write<u64>(state.in_flight.front());
                }

                util::log(LOG_CORE_PIPE1, "CO.", dec_u<0>, slot, ":   Committed uop ", *cur_op);           

                state.commited_micro++;
                if(cur_op->control & mop_last) state.commited_macro++;
            }
            else
            {
                util::log(LOG_CORE_PIPE1, "CO.", dec_u<0>, slot, ":   ROB head not ready to commit.");
                break;
            }
        }
        catch(const LatchEmptyException& le)
        {
            util::log(LOG_CORE_PIPE1, "CO.", dec_u<0>, slot, ":   ROB ", le.what(), " No uop committed.");

            // commit last condition, since no later uop will update this
            std::memcpy(&state.arf->cc, &prf.cc[rrt.cc_lastused.front()], CCREG_SIZE);

            if(!(state.active & ex_active)) next_inactive |= co_active;
            break;
        }
        catch(const LatchStallException& ls)
        {
            // ignore it, this will only occur when store raises an exception
        }
        // catch store exceptions here, flush and inject 
        catch(const MemoryManagerException& mme)
        {
            util::log(LOG_CORE_PIPE1, "CO.", dec_u<0>, slot, ":   MMU: ", mme.what());
            flush();
            // TODO LATENCY
            rob->push_front((state.cycle + 1), { MM::zero_mref, { uop_int, 0, {0}, setExcept(ex_PF, 0) },
                state.cycle + 0 /*latency here*/, setExcept(ex_PF, 0), exec_running, 0, 0 });
        }
    }

    if(rob->empty() && (next_inactive & is_active))
        state.active &= ~is_active & ~ex_active & ~co_active;

    util::log(LOG_CORE_PIPE1, "");
    return 0;
}



RSPort::RSPort(u8 id, vector<u8> types) : id(id), busy(fu_ready)
{
    u8 i = 0;
    for(u8 t : types) fus.push_back(FUInfo(t, i++));
}

// filter rs ports using portmask
vector<RSPort*> Core::get_rsports(u8 portmask)
{
    vector<RSPort*> ret;
    u8 i = 0x01;
    for(RSPort& port : rs.ports)
    {
        if(portmask & i) ret.push_back(&port);
        i <<= 1;
    }
    return ret;
}

std::ostream& operator<<(std::ostream& os, const MM::MemoryRef& mr)
{
    os << "v." << hex_u<64> << mr.vaddr << " size " << dec_u<0> << mr.size << " r " << +mr.ready << " m "
       << MM::memref_mode_str[mr.mode];

    return os;
}

std::ostream& operator<<(std::ostream& os, const ROBEntry& re)
{
    os << re.op  << " | c:" << !!re.c_ready << " | x:"
       << +re.in_exec << " | ex:" << !!re.except
       << " | m:" << MM::memref_mode_str[re.mref.mode];

    return os;
}

std::ostream& operator<<(std::ostream& os, const FUInfo& fui)
{
    uop op = fui.re ? fui.re->op : zero_op;
    os << op << " | t:" << std::setfill(' ') << std::setw(5) << std::left
       << fu_type_str[fui.type] << " | busy:" << !!fui.cycle /*!!fui.busy*/;
    //    << " | id:" << +fui.id;

    return os;
}

std::ostream& operator<<(std::ostream& os, const ReservationStation& rs)
{
    for(auto rsp : rs.ports)
        for(auto fu : rsp.fus)
            os << "RS" << +rsp.id << +fu.id << ":   " << fu << "\n";

    return os;
}

// print n rob entries
std::stringstream Core::rob_readable(u8 n)
{
    std::stringstream ss;

    u32 i = 0;
    try
    {
        for(;i < n && i < rob->size(); i++)
        {
            ROBEntry re = rob->at(state.cycle, i);
            ss << dec_u_lf<2> << i << " |    " << re << "\n";
        }
    }
    catch(const LatchException& le) {}

    for(;i < n; i++)
        ss << dec_u_lf<2> << i << " |    " << zero_re << " **\n";

    return ss;
}

// print specified prf: 0 gp, 1 fp, 2 vr, 3 cc
std::stringstream Core::prf_readable(u8 regclass)
{
    std::stringstream ss;

    if(regclass == 0) // gp
        for(u16 i = 0; i < REGCLS_0_RNREG; i++)
            ss << "p" << dec_u<3> << i << " " << hex_u<REGCLS_0_SIZE*8>
               << prf.gp[i] << (i % 4 == 3 ? "\n" : " ");

    if(regclass == 1) // fp
        for(u16 i = 0; i < REGCLS_1_RNREG; i++)
            ss << "p" << dec_u<3> << i << " " << hex_u<REGCLS_1_SIZE*8>
               << prf.fp[i] << (i % 2 == 1 ? "\n" : " ");

    if(regclass == 2) // vr
        for(u16 i = 0; i < REGCLS_2_RNREG; i++)
            ss << "p" << dec_u<3> << i << " " << hex_u<REGCLS_2_SIZE*8>
               << prf.fp[i] << "\n";

    if(regclass == 3) // cc
        for(u16 i = 0; i < CCREG_CNT; i++)
            ss << "c" << dec_u<3> << i << " " << hex_u<CCREG_SIZE*8>
               << prf.cc[i] << (i % 4 == 3 ? "\n" : " ");

    return ss;
}
