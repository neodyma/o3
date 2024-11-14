// o3 RISC simulator
//
// general configuration
//
// Lukas Heine 2021

#ifndef SIM_CONF_H
#define SIM_CONF_H

static_assert((__x86_64__));

#include "core/cconf.hh"
#include "frontend/fconf.hh"


// simulator config
#define MAX_CYCLES      UINT64_MAX          // max cycles before halt, debug use
#define UQUEUE_SIZE     128                 // number of uops in the uQueue
#define SILENT_HALT     1                   // stop execution without exception if control runs into unmapped addrs

// memory config
#define ADDR_SIZE       64                  // don't change this
#define ADDR_BITS       48                  // implemented bits for canonical addresses
#define PADDR_LIMIT     0xffffffff00000000  // highest frame address
#define VADDR_LIMIT     PADDR_LIMIT         // highest virtual address
#define PAGE_SIZE       4096                // virtual page size in bytes (= frame size)
#define PAGE_MASK       ~(PAGE_SIZE - 1)    // usable frame number bits

// these should be matched to a page boundary..
#define MM_KERNEL_START 0x1000              // kernel
#define MM_USER_START   0x8000              // userspace

#define MM_LOAD_REORDER 1                   // allow reordering of loads

#define MM_ST_LATENCY   0                   // store latency into memory
#define MM_LD_LATENCY   0                   // load latency from memory

// #define L1_ST_LATENCY   0                   // ..

#define STACK_START     0x100000
#define STACK_SIZE      16384


// logging
#define LOG_SIM_INIT    1                   // log parameters
#define LOG_STATE_PRE   3                   // log sim.state before cycle
#define LOG_STATE_POST  3                   // log sim.state after cycle

#define LOG_MM_INIT     1                   // log parameters
#define LOG_MM_MAPPED   2                   // mappings
#define LOG_MM_EXEC     3                   // executed memory requests
#define LOG_MM_REQUEST  3                   // added requests

#define LOG_ALWAYS      0                   // debug only

static_assert((ADDR_SIZE == 64),                "Unsupported vaddr size");
static_assert((bits_set(PAGE_SIZE) == 1),       "Unsupported page size.");
static_assert((VADDR_LIMIT >= MM_USER_START));

#define BANNER_STRING   "//        ________\n//  ________|__  /\n//  _  __ \\__\
/_ < \n//  / /_/ /___/ / \n//  \\____//____/  \n//                "

#endif // SIM_CONF_H
