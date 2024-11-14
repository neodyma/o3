// o3 RISC simulator
//
// branch prediction
// - predictor 1
// - predictor 2
// ..
//
// Lukas Heine 2021

#include "bp.hh"

// always predict next uop
u64 SimplePredictor::predict(u64 rip, u64 seq, u64 target)
{
    (void) rip;
    (void)target;
    return seq;
}

void SimplePredictor::update(u64 rip, u64 target, u8 taken)
{
    (void) rip;
    (void) target;
    (void) taken;
}

u64 BTBPredictor::predict(u64 rip, u64 seq, u64 target)
{
    if(!btb.count(rip))
        return (rip < target ? seq : target); // backward taken, forward not taken
    
    else return btb.at(rip);
}

void BTBPredictor::update(u64 rip, u64 target, u8 taken)
{
    if(taken && btb.size() < BTB_SIZE)
        btb.insert_or_assign(rip, target);

    else if(!taken && btb.count(rip))
        btb.erase(rip);

    util::log(LOG_BP_ALL, "BP__:   Updated branch at ", hex_u<64>, rip, " as ", (taken ? "taken" : "not taken"), ".");
}
