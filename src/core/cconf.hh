// o3 RISC simulator
// 
// core config
//
// Lukas Heine 2021

#ifndef SIM_CCONF_H
#define SIM_CCONF_H

// widths and sizes should match if a 'true' latch behavior (in = out) is expected
#define DECODE_WIDTH    6                   // instructions decoded each cycle
#define DECODE_LATENCY  1                   // amount of cycles until instruction gets placed into uqueue
#define ID_RA_SIZE      6                   // logical number of uops in the decode/rename latch

#define ALLOC_WIDTH     6                   // alloc'd each cycle
#define ALLOC_LATENCY   1                   // latency before uop gets added to ROB
#define ROB_SIZE        224                 // uops in ROB

#define ISSUE_WIDTH     8                   // issued each cycle
#define ISSUE_LATENCY   0                   // latency before uop is ready to execute
#define ISSUE_DEPTH     97                  // ROB entries which are searched to be issued ("RS entries")

#define WB_LATENCY      1                   // cycles until executed uop is ready at ROB

#define COMMIT_WIDTH    6                   // max commits per cycle

// functional units
#define RS_PORTS        8                   // ports from which uops are issued

#define COMMIT_MACRO    0                   // only commit entire macro bundles
#define FAST_EXCEPT     1                   // instantly handle exception (don't jump to handler)

#define LOAD_WIDTH      4                   // loads executed each cycle
#define LQUEUE_SIZE     ROB_SIZE            // number of entries in the load queue

// registers
// register classes should be linked to uop.opcode.prefix
// archreg count per class is limited to 256! (uop.regs) 
// GP registers
#define REGCLS_0_SIZE   8                   // size of gp register class in bytes
#define REGCLS_0_CNT    36                  // number of architectural gp registers
#define REGCLS_0_RNREG  180                 // number of physical (renamed) gp registers

// FP registers
#define REGCLS_1_SIZE   sizeof(long double)
#define REGCLS_1_CNT    16
#define REGCLS_1_RNREG  64

// vector registers
#define REGCLS_2_SIZE   64                  // 512-bit vector registers
#define REGCLS_2_CNT    32
#define REGCLS_2_RNREG  128

// condition registers
#define CCREG_SIZE      8                   // rflags
#define CCREG_CNT       32                  // condition regs are not renamed


// logging
#define LOG_CORE_ARF    1                   // log ARF after each cycle
#define LOG_CORE_BUF    3                   // buffers and queues
#define LOG_CORE_INIT   1                   // log parameters
#define LOG_CORE_PIPE1  2                   // pipeline detail level 1
#define LOG_CORE_PIPE2  4                   // pipeline detail level 2
#define LOG_CORE_PIPE3  5                   // detail level 3
#define LOG_CORE_PRF    7                   // log PRF after each cycle 
#define LOG_CORE_UOP    6                   // uop output

static_assert(COMMIT_MACRO == 0);           // still needs work
static_assert(FAST_EXCEPT  == 1);           // no handlers yet

static_assert(REGCLS_0_SIZE  >= 2);
static_assert(REGCLS_0_RNREG >= REGCLS_0_CNT, "Too few physical registers.");
static_assert(REGCLS_1_RNREG >= REGCLS_1_CNT, "Too few physical registers.");
static_assert(REGCLS_2_RNREG >= REGCLS_2_CNT, "Too few physical registers.");

#endif