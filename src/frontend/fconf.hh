// o3 RISC simulator
// 
// frontend config
// - RISC-like
// - x64
//
// Lukas Heine 2021

#ifndef SIM_FCONF_H
#define SIM_FCONF_H

#define FETCH_WIDTH     4                   // RISC instructions fetched each cycle
#define FETCH_LATENCY   1                   // fetch + bp latency

// branch prediction
#define BTB_SIZE        4096

// x64 config
#define X64_FETCH_BYTES 16                  // # of bytes read from memory and sent to predecode
#define X64_FETCH_ALIGN ~(X64_FETCH_BYTES - 1)

#define IQUEUE_SIZE     50                  // # of x64 instructions in the instruction queue 

#define PD_LATENCY      1


// logging
#define LOG_FE_INIT     1
#define LOG_FE_FETCH    5

#define LOG_BP_ALL      3

#define LOG_64_BUF      3
#define LOG_64_PIPE1    2
#define LOG_64_PIPE2    3
#define LOG_64_PIPE3    6

static_assert((bits_set(X64_FETCH_BYTES) == 1));

#endif