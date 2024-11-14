// o3 RISC simulator
//
// utility
// - operators
// - logging
// - type conversions
//
// Lukas Heine 2021

#ifndef SIM_UTIL_H
#define SIM_UTIL_H

#include "types.hh"

#include <deque>
// #include <iomanip>
#include <iostream>
#include <map>
#include <time.h>


#define asm(...)      __asm__ volatile(__VA_ARGS__)

#define getx64flags "; pushfq; pop %[ccs];" // flag target should be labeled ccs..
#define setx64flags "; push %[ccu]; popfq;" // and source should be labeled ccu

#define bit(n, x)   (((x) >> n)   & 0x01)
#define byte(n, x)  (((x) >> n*8) & 0xff)
#define bits_set(x) __builtin_popcountll(x)
#define b2u8(x)     std::to_integer<u8>(x)
#define bitmask(n)  (~(((~0ull) << ((n)-1)) << 1))
#define sx(x, f, t) (((i64)((x) << (8-f)*8) >> (8-f)*8) & bitmask(t*8))

#define outfile     std::cout
#define errorfile   std::cerr

#define TODO        throw NotImplementedException()
#define HLINE       "------------------------------------------------------------------------------\
----------------------"
#define H2LINE      "==============================================================================\
======================"

// global vars
extern u8 loglevel;

// operators and streams
std::ostream& operator<<(std::ostream& os, const vector<u8>& bytevec);
std::ostream& operator<<(std::ostream& os, const uop& uop);

std::ostream& operator<<(std::ostream& os, const w8& val);
std::ostream& operator<<(std::ostream& os, const w16& val);
std::ostream& operator<<(std::ostream& os, const w32& val);
std::ostream& operator<<(std::ostream& os, const w64& val);

std::ostream& operator<<(std::ostream& os, const u128& val);
std::ostream& operator<<(std::ostream& os, const i128& val);

std::stringstream uop_readable(const uop& uop);
std::stringstream regcls_readable(const u8& opclass);

// pretty print hex with N bits
template<u32 N>
std::ostream& hex_u(std::ostream& os)
{   // don't forget to promote uchar to int or there won't be visible output
    return os << std::hex << std::setw(N / 4) << std::right << std::setfill('0');
}

// decimal with N digits
template<u32 N>
std::ostream& dec_u(std::ostream& os)
{   // don't forget to promote uchar to int or there won't be visible output
    return os << std::dec << std::setw(N) << std::left << std::setfill(' ');
}

template<u32 N>
std::ostream& dec_u_lf(std::ostream& os)
{   // don't forget to promote uchar to int or there won't be visible output
    return os << std::dec << std::setw(N) << std::setfill('0') << std::right;
}

// str with set width and no fill
template<u32 N>
std::ostream& str_w(std::ostream& os)
{
    return os << std::setw(N) << std::setfill(' ') << std::left;
}

namespace util
{
    // abort with error message
    template<class... T> inline
    void abort(T... str) { (errorfile << ... << str) << "\n"; exit(EXIT_FAILURE); }

#ifdef nolog
    template<class... T> inline
    void log(u8 lv, T... str) { (void)(lv); ((void)(str),...); }
    
    template<class... T> inline
    void log_always(T... str) { (outfile << ... << str) << "\n"; }
#else
    // log message depending on loglevel
    template<class... T> inline
    void log(u8 lv, T... str) { if(lv <= loglevel) (outfile << ... << str) << "\n"; }
    
    // always log message
    template<class... T> inline
    void log_always(T ... str) { log(0, str ...); }
#endif // nolog

    vector<u8> str2vec(string& str);
    int parseargs(int argc, char** argv, struct opts* opts);

    // log2
    const std::map<u16, u8> ld
    {
        { 0x0000, 0x00 },
        { 0x0001, 0x00 }, { 0x0002, 0x01 }, { 0x0004, 0x02 }, { 0x0008, 0x03 },
        { 0x0010, 0x04 }, { 0x0020, 0x05 }, { 0x0040, 0x06 }, { 0x0080, 0x07 },
        { 0x0100, 0x08 }, { 0x0200, 0x09 }, { 0x0400, 0x0a }, { 0x0800, 0x0b },
        { 0x1000, 0x0c }, { 0x2000, 0x0d }, { 0x4000, 0x0e }, { 0x8000, 0x0f },
    };

    // gnu.org/software/libc/manual/html_node/Calculating-Elapsed-Time.html
    u8 timediff(timespec* res, timespec* start, timespec* end);
} // util

#endif // SIM_UTIL_H