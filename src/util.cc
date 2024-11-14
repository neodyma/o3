// o3 RISC simulator
//
// utility
// - operators
// - logging
// - type conversions
//
// Lukas Heine 2021

#include <algorithm>
#include <bit>
#include <fstream>
// #include <elf.h>

#include "cxxopts.hh"

#include "util.hh"
#include "sim.hh"
#include "core/uops.hh"

namespace util
{

// "a8ef.." -> [0xa8, 0xef, ...]
// see https://stackoverflow.com/a/30606613/9958527
// man 3 endian
vector<u8> str2vec(std::string& str)
{
    // remove comments
    for(size_t a = str.find("#"), b = str.find("\n", a);
        a != std::string::npos && b != std::string::npos;)
    {
        str.erase(a, b-a+1);
        a = str.find("#");
        b = str.find("\n", a);
    }

    // remove remaining whitespace and newlines
    str.erase(std::remove(str.begin(), str.end(), ' '), str.end());
    str.erase(std::remove(str.begin(), str.end(), '\n'), str.end());

    if((str.length() % 2 != 0) ||
        str.find_first_not_of("0123456789abcdefABCDEF", 0) != std::string::npos)
            return std::vector<u8>();

    std::vector<u8> bytes;
    bytes.reserve(str.length()/2);

    for(uint64_t i = 0; i < str.length(); i += 2)
    {
        std::string substr = str.substr(i, 2);
        u8 byte = (u8) strtoul(substr.c_str(), NULL, 16);
        bytes.push_back(byte);
    }

    return bytes;
}

// parse args to struct
int parseargs(int argc, char** argv, struct opts* myopts)
{
    // tbd use getopt instead, compile is kinda slow
    // parse options, set vars
    cxxopts::Options options("o3.x", "o3 core simulator");

    options.custom_help("[options ..];  ./o3.x -v -i examples/o3/loop.bin");

    options.add_options()
        // option,                  description,                            default val
        ("l,loglv",             "set loglevel from 0-7",    cxxopts::value<u8>()->default_value("0")            )
        ("v,verbose",           "\"-l 7\"",                 cxxopts::value<bool>()->default_value("false")      )
        ("m,mcode",             "machine code",             cxxopts::value<std::string>()                       )
        ("i,infile",            "path to input file",       cxxopts::value<std::string>()                       )
        // Data?
        ("t,time",              "measure simulation time"                                                       )
        ("f,frontend",          "select frontend",          cxxopts::value<std::string>()->default_value("risc"))
        ("h,help",              "print help"                                                                    )
        ;

    #ifdef simtest
    options.add_options()
        ("t,test",              "start googletest"                                                              )
        ;
    options.allow_unrecognised_options();
    #endif // simtest

    cxxopts::ParseResult opts;
    try { opts = options.parse(argc, argv); }
    catch (cxxopts::OptionParseException& e) { util::abort(e.what()); }

    #ifdef simtest
    // this *will* ignore all other flags
    if(opts.count("test"))
        { ::testing::InitGoogleTest(&argc, argv); return RUN_ALL_TESTS(); }
    #endif // simtest

    if(opts.count("help") || argc == 1)
        { std::cout << BANNER_STRING << options.help() << std::endl; exit(EXIT_SUCCESS); }

    // loglevels
    // 0: nothing   1: ..   2: ..   3: ..   4: ..   5: ..   6: ..   7: everything
    loglevel = opts["loglv"].as<uint8_t>();
    if (loglevel > 7 || opts["verbose"].as<bool>()) loglevel = 7;

    myopts->time = opts.count("time");
    
    // machine code
    if(!(opts.count("mcode")) && !(opts.count("infile")))
        util::abort("mcode or infile are required to run. Use -h for help.");
    else if(!(opts.count("infile")))
    {
        std::string mstr = opts["mcode"].as<std::string>();
        myopts->code = util::str2vec(mstr);
        if(myopts->code.empty()) util::abort("Machine code is not valid.");
    }
    else // read from file
    {
        std::ifstream infile (opts["infile"].as<std::string>());
        if(!infile) util::abort("File could not be opened.");
        std::string mstr;
        infile.seekg(0, std::ios::end);
        mstr.resize(infile.tellg());
        infile.seekg(0, std::ios::beg);
        infile.read(mstr.data(), mstr.size());
        infile.close();
        myopts->code = util::str2vec(mstr);
        if(myopts->code.empty()) util::abort("Machine code is not valid.");
    }
    
    // frontend select, default to risc
    std::string fstr = opts["frontend"].as<std::string>();
    myopts->frontend = strcmp(fstr.c_str(), "x64") ? risc : x64;

    return 0;
}

// template<class T, size_t N>
// T* AlignedAllocator<T, N>::allocate(size_t n)
// {
//     if(!n) return NULL;
//     void* const p = aligned_alloc(N, n * sizeof(T));
//     if(!p) throw std::bad_alloc();

//     return static_cast<T*>(p);
// }

// template<class T, size_t N>
// void AlignedAllocator<T, N>::deallocate(T* p, size_t n)
// {
//     (void) n;
//     free(p);
// }

// store start-end in res
u8 timediff(timespec* res, timespec* start, timespec* end)
{
    if(end->tv_nsec < start->tv_nsec)
    {
        u32 psec = (start->tv_nsec - end->tv_nsec) / 1000000000 + 1;
        start->tv_nsec -= 1000000000 * psec;
        start->tv_sec  += psec;
    }
    if(end->tv_nsec - start->tv_nsec > 1000000)
    {
        u32 psec = (end->tv_nsec - start->tv_nsec) / 1000000000;   
        start->tv_nsec += 1000000000 * psec;
        start->tv_sec  -= psec;
    }
    res->tv_nsec = end->tv_nsec - start->tv_nsec;
    res->tv_sec  = end->tv_sec  - start->tv_sec;

    return (end->tv_sec < start->tv_sec);
}

} // util

// print bytes from bytevector
std::ostream& operator<<(std::ostream& os, const std::vector<u8>& bytevec)
{
    for(u8 b: bytevec) os << hex_u<8> << +b << " ";

    return os;
}

std::ostream& operator<<(std::ostream& os, const w8& val)
{
    os << hex_u<8> << +std::to_integer<u8>(val.b);
    return os;
}

std::ostream& operator<<(std::ostream& os, const w16& val)
{
    os << hex_u<16> << std::bit_cast<u16>(val);
    return os;
}

std::ostream& operator<<(std::ostream& os, const w32& val)
{
    os << hex_u<32> << std::bit_cast<u32>(val);
    return os;
}

std::ostream& operator<<(std::ostream& os, const w64& val)
{
    os << hex_u<64> << std::bit_cast<u64>(val);
    return os;
}

// hex only
std::ostream& operator<<(std::ostream& os, const u128& val)
{
    os << hex_u<64> << (u64)(val >> 64) << hex_u<64> << (u64)(val & UINT64_MAX);
    return os;
}

std::ostream& operator<<(std::ostream& os, const i128& val)
{
    os << hex_u<64> << (u64)((u128)val >> 64) << hex_u<64> << (u64)(val & UINT64_MAX);
    return os;
}
