// o3 RISC simulator
// 
// memory management
// - virtual memory
// - general memory management
//
// Lukas Heine 2021

#ifndef SIM_MEM_H
#define SIM_MEM_H

#include "sim.hh"
#include "conf.hh"
#include "util.hh"

#include <tuple>
#include <map>

#define toAlignedAddr(x)    ((x) << __builtin_ctzll(PAGE_SIZE))     // shift in zero bits to align
#define pageFloor(x)        ((x) & PAGE_MASK)                       // aligned page address bits
#define pageCeil(x)         pageFloor((x) + PAGE_SIZE)              // start of next page
#define pageOffs(x)         ((x) & ~PAGE_MASK)                      // page offset bits
#define pageCnt(x)          pageCeil((x) - 1) / PAGE_SIZE           // needed pages for x bytes

namespace MM
{
    // page frame and flags
    struct PageFrame 
    {
        void* const data;       // actual bytes
        size_t      bytes_used; // need this for partially mapped frames
        i8          pl;
        u8          rwx;        
        u8          ext;        // page references external memory
        string      name;       // page description, section, ..
    }; // PageFrame

    // frame number and flags
    struct PageTableEntry
    {
        u64 frameno;
        u8  present;
        i8  pl;
        u8  rwx;
        // u8  dirty;
        // u8  accessed;
        // u8 enable_caching;
    }; // PageTableEntry

    // typedef enum rwx_bits;
    // -> mem.tt

    typedef enum
    {
        mr_invalid, // no memory reference
        mr_read,    // r
        mr_write,   // w
        mr_branch,  // branch target
        mr_rel,     // rip relative
    } memref_mode;

    typedef enum
    {
        mr_unavail,  // not ready
        mr_exready,  // ready for execution
        mr_inexec,   // in execution
        mr_valready, // request completed
    } memreq_status;

    // TODO add a local buffer for data, since it might be overwritten with long store latencies

    // required info for load/store
    struct MemoryRef
    {
        void*  data  = nullptr;    // value to r/w
        size_t size  = 0;          // #bytes
        u64    vaddr = 0;          // vaddr to access
        u8     mode  = mr_invalid; // 0/r/w/b/br
        u8     ready = mr_unavail; // r/w complete
    }; // MemoryRef

    const MemoryRef zero_mref = { nullptr, 0, 0, 0, 0 };

    const std::string memref_mode_str[5] = { ("0"), ("r"), ("w"), ("b"), ("+") };

    struct MemoryRequest
    {
        MemoryRef* mref      = nullptr; // careful, store reference might be gone from ROB
        u32*       exception = nullptr;
        u64        cycle     = 0;
    }; // MemoryRequest

    struct StoreRequest
    {
        MemoryRef  mref      = zero_mref; // no pointer!
        u64        cycle     = 0;
    }; // MemoryRequest

    // vaddr is in canonical form
    constexpr u8 is_canonical(u64 vaddr)
    {
        if((vaddr >> (ADDR_BITS - 1)) & 0b1) // "negative address"
            return bits_set((vaddr >> ADDR_BITS)) == (ADDR_SIZE - ADDR_BITS);
        else // "positive address"
            return bits_set((vaddr >> ADDR_BITS)) == 0;
    }
} // MM

class MemoryManager
{
    public:
    MemoryManager(Simulator::SimulatorState& state);
    ~MemoryManager();
    u8 refresh();
    u8 clear_bufs();
    u8 active();

    MM::PageFrame&      map_frame(u64 paddr, i8 pl, u8 rwx, string name);
    u8                  unmap_frame(u64 paddr);
    u8                  unmap_all_frames();
    
    MM::PageTableEntry& map_page(u64 vaddr, u64 paddr, u8 present, i8 pl, u8 rwx);
    u8                  unmap_page(u64 vaddr);
    u8                  unmap_all_pages();

    vector<std::pair<MM::PageFrame*, u64>> mmap_frames(u64 paddr, void* extaddr, size_t len, i8 pl,
        u8 rwx, string name);

    u8    is_busy(u64 vaddr, size_t len);
    u8    is_alias(u64 vaddr1, size_t len1, u64 vaddr2, size_t len2);
    u8    bad_rwx(u64 vaddr, size_t len, u8 rwx);
    u8    bad_pl(u64 vaddr, size_t len);

    u64   get_paddr(u64 vaddr, u8 rwx);
    void* get_eaddr(u64 vaddr, u8 rwx);

    u8    get(MM::MemoryRequest& req, u8 rx);
    u8    put(MM::MemoryRequest& req);

    template<typename T> std::pair<T, u64> read(u64 vaddr, u8 rx);
    template<typename T> T                 readT(u64 vaddr, u8 rx);
    pair<u64, u64>                         read(u64 vaddr, void* data, size_t len, u8 rx);
    
    template<typename T> void              write(u64 vaddr, T val);
    void                                   write(u64 vaddr, void* data, size_t len);

    private:
    std::map<u64, MM::PageTableEntry> pagetable; // unified page table
    std::map<u64, MM::PageFrame>      mem;       // "lazy" memory

    std::deque<MM::MemoryRequest>     ldbuf;     // use to enfore load/store latencies
    std::deque<MM::StoreRequest>      stbuf;     // all requests in the store buffer *must* be executed

    Simulator::SimulatorState&        state;
}; // MemoryManager

// memory operator overloads
std::ostream& operator<<(std::ostream& os, const MM::PageFrame& pf);
std::ostream& operator<<(std::ostream& os, const MM::PageTableEntry& pte);

struct MemoryManagerException : public SimulatorException {};

struct AllocationFailedException : public MemoryManagerException
{
    const char* what () const throw ()
    {   return "memory allocation failed."; }
}; // AllocationFailedException

struct InvalidPageaddrException : public MemoryManagerException
{
    const char* what () const throw ()
    { return "page start address not valid."; }
}; // InvalidPageaddrException

struct InvalidAddrException : public MemoryManagerException
{
    const char* what () const throw ()
    { return "address not valid."; }
}; // InvalidAddrException

struct InvalidMemoryLocationException : public MemoryManagerException
{
    const char* what () const throw ()
    { return "memory location does not exist."; }
}; // InvalidMemoryLocationException

struct PageNotMappedException : public MemoryManagerException
{
    const char* what () const throw ()
    { return "page is not mapped."; }    
}; // PageNotMappedException

struct PageAlreadyMappedException : public MemoryManagerException
{
    const char* what () const throw ()
    { return "address already mapped."; }    
}; // PageAlreadyMappedException

struct PageNotPresentException : public MemoryManagerException
{
    const char* what () const throw ()
    { return "data not present in memory."; }    
}; // PageNotPresentException

struct ProtectionViolationException : public MemoryManagerException
{
    const char* what () const throw ()
    { return "page access protection violated."; }
}; // ProtectionViolationException

struct AccessBitViolationException : public MemoryManagerException
{
    const char* what () const throw ()
    { return "access permissions do not match."; }
}; // AccessBitViolationException

struct AlignmentViolationException : public MemoryManagerException
{
    const char* what () const throw ()
    { return "required address alignment violated."; }    
}; // AlignmentViolationException

#include "mem.tt"

#endif // SIM_MEM_H
