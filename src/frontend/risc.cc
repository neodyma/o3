// o3 RISC simulator
//
// frontends
//
// Lukas Heine 2021

#include "frontend.hh"
#include "fconf.hh"

#include "../core/uops.hh"

#include <endian.h>

RiscFrontend::RiscFrontend(MemoryManager& mmu, LatchQueue<uop>* uqueue,
        Simulator::SimulatorState& state) : Frontend(mmu, uqueue, state)
{
    bp = new BTBPredictor();

    util::log(LOG_FE_INIT, "RISC frontend initialized.\n");
}

RiscFrontend::~RiscFrontend()
{
    delete bp;
}

// fetch only
u8 RiscFrontend::cycle()
{
    // todo fetch traces instead of sequential ops? -> trace cache in bp
    // check status
    if( !(state.active & if_active) )
    {
        util::log(LOG_FE_FETCH, "IF__:   Frontend inactive.\n");
        return 1;
    }

    util::log(LOG_FE_FETCH, "IF__:   Fetching new instructions from memory.");

    std::pair<uop, u64> fetch = { zero_op, 0 };
    uop cur_op = zero_op;

    for(u64 slot = 0; slot < FETCH_WIDTH; slot++)
    {
        if(uqueue->size() >= UQUEUE_SIZE)
        {
            util::log(LOG_FE_FETCH, "IF__: * uQ is full. Not fetching any instructions.");
            break;
        }

        try
        {
            util::log(LOG_FE_FETCH, "IF__:   Fetchaddr: ", hex_u<64>, fetchaddr);

            if(!mmu.is_busy(fetchaddr, sizeof(uop)))
                fetch = mmu.read<uop>(fetchaddr, MM::p_x);
            else
            {
                util::log(LOG_FE_FETCH, "IF__:   Waiting for memory..");
                break;
            }

            cur_op = fetch.first;

            cur_op.opcode  = be16toh(cur_op.opcode);
            cur_op.control = be16toh(cur_op.control) | mop_first | mop_last; // all uops are standalone here!
            cur_op.imm     = be64toh(cur_op.imm);

            util::log(LOG_FE_FETCH, "IF__:   Fetched instruction ", cur_op, ".");
        }
        catch(const InvalidAddrException& ia)
        {   // read after last mapped byte
            state.active &= ~fe_active;

            // "instruction" after the last instruction
            if(SILENT_HALT && fetchaddr == state.in_flight.back())
            {
                util::log(LOG_FE_FETCH, "IF__:   End of code reached.");
                break;
            }
            else // #PF
            {
                cur_op.opcode      = uop_int;
                cur_op.control     = mop_first | mop_last | use_imm;
                *(u32*)cur_op.regs = 0;
                cur_op.imm         = setExcept(ex_PF, expf_present | expf_ifetch |
                    (state.ring == pl_user ? expf_user : 0));
            }
        }
        catch(const ProtectionViolationException& pv)
        {
            util::log(LOG_FE_FETCH, "IF__:   Fetch ", pv.what(), " Injecting #PF.");
            cur_op.opcode      = uop_int;
            cur_op.control     = mop_first | mop_last | use_imm;
            *(u32*)cur_op.regs = 0;
            cur_op.imm         = setExcept(ex_PF, expf_present | expf_ifetch |
                (state.ring == pl_user ? expf_user : 0));
        }
        catch(const PageNotMappedException& pm)
        {
            util::log(LOG_FE_FETCH, "IF__:   Fetch ", pm.what(), " Injecting #PF.");
            cur_op.opcode      = uop_int;
            cur_op.control     = mop_first | mop_last | use_imm;
            *(u32*)cur_op.regs = 0;
            cur_op.imm         = setExcept(ex_PF, expf_present | expf_ifetch |
                (state.ring == pl_user ? expf_user : 0));
        }
   
        u64 seq = fetchaddr + 0x10;
        state.seq_addrs.push_back(seq);
        fetchaddr = (is_branch(cur_op) ? bp->predict(fetchaddr, seq, cur_op.imm) : seq);
        state.in_flight.push_back(fetchaddr); // predicted next instruction

        try
        {
            uqueue->push_back( (state.cycle + FETCH_LATENCY + fetch.second), cur_op );
        }
        catch(const LatchFullException& fe) // this should _never_ occur since size is checked
        {
            util::log(LOG_FE_FETCH, "IF__: * uQ ", fe.what(), " Not fetching any instructions.");
            break;
        }   
    }

    util::log(5, "");
    return 0;
}

u8 RiscFrontend::flush()
{
    return 0;
}
