// o3 RISC simulator
// 
// types and global structs
//
// Lukas Heine 2021

#ifndef SIM_TYPES_H
#define SIM_TYPES_H

#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <exception>
#include <string>
#include <sstream>
#include <vector>

// _should_ be available, it's 2021 after all
static_assert(__SIZEOF_INT128__ != 0);

typedef uint8_t     u8;  
typedef uint16_t    u16;
typedef uint32_t    u32;
typedef uint64_t    u64;
typedef __uint128_t u128;

typedef int8_t      i8;
typedef int16_t     i16;
typedef int32_t     i32;
typedef int64_t     i64;
typedef __int128_t  i128;

typedef float       f32;
typedef double      f64;
typedef long double f80;

typedef std::byte   b8;

// unions are UB, -> c++20 for bitcast?
// https://gist.github.com/shafik/a956a17d00024b32b35634eeba1eb49e

typedef struct { b8 b; }                                               w8;
typedef struct { b8 b[2];   b8& operator[](u8 i)    { return b[i]; } } w16;
typedef struct { b8 b[4];   b8& operator[](u8 i)    { return b[i]; } } w32;
typedef struct { b8 b[8];   b8& operator[](u8 i)    { return b[i]; } } w64;
typedef struct { b8 b[16];  b8& operator[](u8 i)    { return b[i]; } } w128;
typedef struct { b8 b[32];  b8& operator[](u8 i)    { return b[i]; } } w256;
typedef struct { b8 b[64];  b8& operator[](u8 i)    { return b[i]; } } w512;

template<u64 N>
struct wN      { b8 b[N];   b8& operator[](u64 i)   { return b[i]; } } __attribute__((may_alias));

template<typename T>
using wT =          wN<sizeof(T)>;

typedef uintptr_t   uptr;
typedef std::string string;

template<class T>
using vector =      std::vector<T>;

template<class T, class U>
using pair =        std::pair<T, U>;

template<u64 N>
std::ostream& operator<<(std::ostream& os, const wN<N>& word);

struct opts
{
    // this maps the entire bytecode
    vector<u8> code;
    u8 frontend;
    u8 time;
    // ELF
    // Data
} __attribute__((aligned(16))); // opts

struct uop
{
    u16 opcode;
    u16 control;
    u8  regs[4]; // abcd
    u64 imm;
} __attribute__((packed, aligned(16))); // uop

const uop zero_op = { 0, 0, { 0 }, 0 };

// hold values until release condition is met
// values are available at output after latch()
// use as 'latch': #pushed == #popped, else queue
template<typename T>
class LatchQueue
{
    public:
    LatchQueue(u32 max_size);

    int     latch();
    bool    ready(u64 cycle);
    int     latch_firstready(u64 cycle, u64 count);

    bool    empty();
    size_t  size();
    
    int     clear();
    void    push_back(u64 cycle, T elem);
    T&      back();

    void    push_front(u64 cycle, T elem);
    T       get_front(u64 cycle);
    T&      front(u64 cycle);
    void    pop_front();

    T&      at(u64 cycle, u64 index);

    auto begin();
    auto end();

    struct LatchQElem
    {
        u64 cycle;
        T   elem;
    }; // LatchQElem

    private:
    u32                                         max_size;
    std::deque<LatchQElem>                      queue;
    typename std::deque<LatchQElem>::iterator   iter_in, iter_out;
}; // LatchQueue

struct SimulatorException : public std::exception
{
    const char* what () const throw ()
    {   return "unspecified simulator exception."; }
}; // SimulatorException

struct LatchException : public SimulatorException {};

struct LatchEmptyException : public LatchException
{
    const char* what () const throw ()
    {   return "is empty."; }
}; // LatchEmptyException

struct LatchFullException : public LatchException
{
    const char* what () const throw ()
    {   return "is full."; }
}; // LatchFullException

struct LatchStallException : public LatchException
{
    const char* what () const throw ()
    {   return "content is not ready."; }
}; // LatchStallException

struct OpcodeHasher
{
    public:
    size_t operator()(std::vector<u8> const& v) const
    {
        size_t hash = 0;
        for(auto& elm : v)
            hash = (hash << 8) | elm;
        return hash;
    }
};


#include "types.tt"

#endif // SIM_TYPES_H
