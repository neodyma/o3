// o3 RISC simulator
//
// branch prediction
// - predictor 1
// - predictor 2
// ..
//
// Lukas Heine 2021

#ifndef SIM_BP_H
#define SIM_BP_H

#include "../util.hh"
#include "fconf.hh"

#include <unordered_map>

class BranchPredictor
{
    public:
    BranchPredictor() {};
    virtual ~BranchPredictor() {};
    virtual u64  predict(u64 rip, u64 seq, u64 target) = 0;
    virtual void update(u64 rip, u64 target, u8 taken) = 0;
}; // BranchPredictor

class SimplePredictor : public BranchPredictor
{
    public:
    SimplePredictor() : BranchPredictor() {};
    ~SimplePredictor() {};
    u64  predict(u64 rip, u64 seq, u64 target);
    void update(u64 rip, u64 target, u8 taken);
};

class BTBPredictor : public BranchPredictor
{
    public:
    BTBPredictor() : BranchPredictor() {};
    ~BTBPredictor() {};
    u64  predict(u64 rip, u64 seq, u64 target);
    void update(u64 rip, u64 target, u8 taken);

    private:
    // all not taken at start
    std::unordered_map<u64, u64> btb;
};

#endif // SIM_BP_H