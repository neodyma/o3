// o3 RISC simulator
// 
// memory management
// - virtual memory
// - heap management
//
// Lukas Heine 2021

#include "mem.hh"
#include "util.hh"

#include "core/uops.hh"

#include <cstring>

MemoryManager::MemoryManager(Simulator::SimulatorState& state) : state(state)
{
    util::log(LOG_MM_INIT, "MMU initialized with:");
    util::log(LOG_MM_INIT, "        ADDR_SIZE ", dec_u<0>, ADDR_SIZE);
    util::log(LOG_MM_INIT, "        PAGE_SIZE ", dec_u<0>, PAGE_SIZE, "\n");
}

MemoryManager::~MemoryManager()
{
    unmap_all_pages();
    unmap_all_frames();
}

// try to execute any pending reads/writes and update buffers
u8 MemoryManager::refresh()
{
    // store queue only contains committed stores, load queue might contain new dependent loads 
    // -> loads in range have to be deferred until all related stores have finished
    //    loads are also checked when a store commits, so we don't have to do any bookkeeping

    util::log(LOG_MM_EXEC, "MMU_:   Executing memory requests..");

    // stores only enter the queue on commit
    for(auto& mr : stbuf)
    {
        if(state.cycle < mr.cycle) break; // stores always in order and never speculative
        write(mr.mref.vaddr, mr.mref.data, mr.mref.size);
        free(mr.mref.data); // this should be a local buffer at this point
        // mr.mref.ready = MM::mr_valready; // irrelevant, store is already commited
        stbuf.pop_front();
    }

    // core::exec sets ready before ::get, that checks any exceptions before adding to ldbuf
    for(auto i = ldbuf.begin(); i != ldbuf.end();)
    {
        auto& mr = *i;
        if(state.cycle < mr.cycle || is_busy(mr.mref->vaddr, mr.mref->size))
        {
            ++i;
            #ifdef MM_LOAD_REORDER
                continue; // proceed with succeeding loads
            #elif
                break;    // or defer them
            #endif
        }
        read(mr.mref->vaddr, mr.mref->data, mr.mref->size, MM::p_r);
        mr.mref->ready = MM::mr_valready;
        i = ldbuf.erase(i); // next element
    }

    return 0;
}

// clear load buffer
u8 MemoryManager::clear_bufs()
{
    ldbuf.clear();
    // don't clear storebuf, all requests are already commited and have to be executed
    return 0;
}

u8 MemoryManager::active()
{
    return !stbuf.empty();
}

// map a zero initialized page frame to physical memory
MM::PageFrame& MemoryManager::map_frame(u64 paddr, i8 pl, u8 rwx, string name)
{
    // check if address is valid or already mapped
    if((paddr > PADDR_LIMIT) || (paddr % PAGE_SIZE)) throw InvalidPageaddrException();
    if(mem.count(paddr)) throw PageAlreadyMappedException();

    MM::PageFrame frame = { aligned_alloc(PAGE_SIZE, PAGE_SIZE), PAGE_SIZE, pl, rwx, 0, name };
    if(!(frame.data)) throw AllocationFailedException();

    memset(frame.data, 0, PAGE_SIZE); // init to zero

    // insert and return reference to frame
    auto ret = mem.insert({ paddr, frame });
    util::log(LOG_MM_MAPPED, "MMU_:   Mapped frame p.", hex_u<64>, paddr, " \'", frame.name, "\' with data at e.",
        hex_u<64>, (uptr)frame.data, ".\n");

    return ret.first->second;
}

// unmap a page frame from physical memory
u8 MemoryManager::unmap_frame(u64 paddr)
{
    if((paddr > PADDR_LIMIT) || (paddr % PAGE_SIZE)) throw InvalidPageaddrException();
    if(!mem.count(paddr)) throw PageNotMappedException();

    MM::PageFrame* cur_page = &mem.at(paddr);
    util::log(LOG_MM_MAPPED, "MMU_:   Unmapped frame p.", hex_u<64>, paddr, " with data at e.", hex_u<64>,
        (uptr)cur_page->data, ".\n");

    free((void*)cur_page->data);
    mem.erase(paddr);

    return 0;
}

// unmap all page frames
u8 MemoryManager::unmap_all_frames()
{
    void* cur_data;
    for(auto& pf : mem)
    {
        if(pf.second.ext)
        {
            cur_data = pf.second.data;
            util::log(LOG_MM_MAPPED, "MMU_:   Freeing e.", (uptr)cur_data, ".");
            free(cur_data);
        }
    }

    util::log(LOG_MM_MAPPED, "MMU_:   Memory cleared.\n");
    mem.clear();

    return 0;
}

// map a page into virtual address space
MM::PageTableEntry& MemoryManager::map_page(u64 vaddr, u64 paddr, u8 present, i8 pl, u8 rwx)
{
    if((vaddr > VADDR_LIMIT) || (vaddr % PAGE_SIZE)) throw InvalidPageaddrException();
    if(pagetable.count(vaddr)) throw PageAlreadyMappedException();

    MM::PageTableEntry pte = { paddr, present, pl, rwx };
    auto ret = pagetable.insert({ vaddr, pte });
    util::log(LOG_MM_MAPPED, "MMU_:   Mapped page v.", hex_u<64>, vaddr, " -> p.", hex_u<64>, paddr, ".");

    return ret.first->second;
}

// unmap page from virtual address
u8 MemoryManager::unmap_page(u64 vaddr)
{
    if((vaddr > VADDR_LIMIT) || (vaddr % PAGE_SIZE)) throw InvalidPageaddrException();
    if(!pagetable.count(vaddr)) throw PageNotMappedException();

    u64 cur_paddr = pagetable.at(vaddr).frameno;
    pagetable.erase(vaddr);
    util::log(LOG_MM_MAPPED, "MMU_:   Unmapped page v.", hex_u<64>, vaddr, " -> p.", hex_u<64>, cur_paddr, ".\n");

    return 0;
}

// remove all virtual address mappings
u8 MemoryManager::unmap_all_pages()
{
    pagetable.clear();
    util::log(LOG_MM_MAPPED, "MMU_:   Page table cleared.\n");

    return 0;
}

// map memory into frames starting at paddr and return references plus physical addresses
// pl and rwx should match the page (for now)
vector<std::pair<MM::PageFrame*, u64>> MemoryManager::mmap_frames(u64 paddr, void* extaddr,
    size_t len, i8 pl, u8 rwx, string name /*, u8 mode*/)
{
    size_t frame_cnt = pageCnt(len);                // needed frames for memory range
    util::log(LOG_MM_MAPPED, "MMU_:   Trying to map ", dec_u<0>, len, " bytes across ", dec_u<0>, frame_cnt,
        " frames.");

    if((paddr > PADDR_LIMIT) || (paddr % PAGE_SIZE))
        throw InvalidPageaddrException();
    
    if(mem.count(paddr))
        throw PageAlreadyMappedException();

    // todo some more checks
    vector<std::pair<MM::PageFrame*, u64>>  mapped; // mapped frames plus paddrs

    u64 cur_paddr = paddr;
    uptr cur_eaddr = (uptr)extaddr;
    for(;len >= PAGE_SIZE; (len -= PAGE_SIZE))
    {        
        MM::PageFrame frame = { (void*)cur_eaddr, PAGE_SIZE, pl, rwx, 1, name };
        auto ins = mem.insert({ cur_paddr, frame });

        util::log(LOG_MM_MAPPED, "MMU_:   Mapped frame \'", frame.name, "\' p.", hex_u<64>, cur_paddr, " -> e.",
            hex_u<64>, (uptr)frame.data, ".");

        mapped.push_back( std::make_pair(&ins.first->second, cur_paddr) );

        cur_eaddr += PAGE_SIZE;
        cur_paddr += PAGE_SIZE; // todo find free frame instead of always using the next one, +check
    }

    // special case: last frame is smaller than frame size
    if(len && (len < PAGE_SIZE))
    {
        MM::PageFrame frame = { (void*)cur_eaddr, len, pl, rwx, 1, name };
        auto ins = mem.insert({ cur_paddr, frame });

        util::log(LOG_MM_MAPPED, "MMU_:   Mapped frame \'", frame.name, "\' p.", hex_u<64>, cur_paddr, " -> e.",
            hex_u<64>, (uptr)frame.data, ".");
        
        mapped.push_back( std::make_pair(&ins.first->second, cur_paddr) );
    }

    return mapped;
}

// check if range from ref.vaddr has any pending writes
u8 MemoryManager::is_busy(u64 vaddr, size_t len)
{
    // this might be a little expensive..
    // though the store queue size is somewhat limited by commit width 
    for(auto& streq : stbuf)
        if(is_alias(vaddr, len, streq.mref.vaddr, streq.mref.size))
            return 1;
    return 0;
}

// check if two vaddr ranges alias to a shared paddr
u8 MemoryManager::is_alias(u64 vaddr1, size_t len1, u64 vaddr2, size_t len2)
{
    //   A-----|
    // |----|----|----|----|
    //    |--------B

    u64 vaddrA  = vaddr1 <= vaddr2 ? vaddr1 : vaddr2; // A is smaller address
    u64 vaddrB  = vaddr1 <= vaddr2 ? vaddr2 : vaddr1;
    size_t lenA = vaddr1 <= vaddr2 ? len1 : len2;
    size_t lenB = vaddr1 <= vaddr2 ? len2 : len1;

    // on same page? check this first due to locality
    if(pageFloor(vaddrA) == pageFloor(vaddrB))
        return (vaddrA + lenA > vaddrB);

    try
    {
        for(u64 i = 0; i < pageCnt(lenA); i += PAGE_SIZE)
        {
            u64 paddr1 = pagetable.at(pageFloor(vaddrA + i)).frameno + pageOffs(vaddrA);
            u64 paddr2 = pagetable.at(pageFloor(vaddrB + i)).frameno + pageOffs(vaddrB);

            u64 paddrA  = paddr1 <= paddr2 ? paddr1 : paddr2; // A is smaller phys address
            u64 paddrB  = paddr1 <= paddr2 ? paddr2 : paddr1;
            size_t len  = paddr1 <= paddr2 ? lenA : lenB;     // len associated with A

            // not on same pageframe
            if(pageFloor(paddrA) != pageFloor(paddrB))
                continue;
            else return (paddrA + len > paddrB);
        }
    }
    catch(const MemoryManagerException& me) { /* not mapped -> no alias */ }
    catch(const std::out_of_range& oor)     { /* not mapped -> no alias */ }

    return 0;
}

// check if range from vaddr can be accessed with given modify bit
u8 MemoryManager::bad_rwx(u64 vaddr, size_t len, u8 rwx)
{   // only check pages for now
    if((rwx & pagetable.at(pageFloor(vaddr)).rwx) &&
       (rwx & pagetable.at(pageFloor(vaddr + len - 1)).rwx))
        return 0;
    else return 1;
}

// check if range from vaddr can be accessed with current protection level
u8 MemoryManager::bad_pl(u64 vaddr, size_t len)
{
    // might need to check pages in between..
    if((state.ring <= pagetable.at(pageFloor(vaddr)).pl) &&
       (state.ring <= pagetable.at(pageFloor(vaddr + len - 1)).pl))
        return 0;
    else return 1;
}

// get paddr for a given vaddr
u64 MemoryManager::get_paddr(u64 vaddr, u8 rwx)
{
    if(!pagetable.count(pageFloor(vaddr))) throw PageNotMappedException();

    if(!(rwx & pagetable.at(pageFloor(vaddr)).rwx))
        throw AccessBitViolationException();

    if(state.ring > pagetable.at(pageFloor(vaddr)).pl)
        throw ProtectionViolationException();

    return pagetable.at(pageFloor(vaddr)).frameno + pageOffs(vaddr);
}

// resolve vaddr to frame+offset and return pointer to data
void* MemoryManager::get_eaddr(u64 vaddr, u8 rwx)
{
    if(!pagetable.count(pageFloor(vaddr))) throw PageNotMappedException();

    u64 frame = pageFloor(get_paddr(vaddr, rwx));

    if(!mem.count(frame) || (pageOffs(vaddr) > (mem.at(frame).bytes_used - 1)))
        throw InvalidAddrException();

    if(!(rwx & mem.at(frame).rwx))
        throw AccessBitViolationException();

    if(state.ring > mem.at(frame).pl)
        throw ProtectionViolationException();

    return (void*)((uptr)mem.at(frame).data + pageOffs(vaddr));
}

// add a read request to the load queue
u8 MemoryManager::get(MM::MemoryRequest& req, u8 rx)
{
    u8 present = !!(pagetable.count(pageFloor(req.mref->vaddr)) &&
        pagetable.count(pageFloor(req.mref->vaddr + req.mref->size - 1)));
    
    if(!present || bad_pl(req.mref->vaddr, req.mref->size) || bad_rwx(req.mref->vaddr, req.mref->size, rx))
    {
        util::log(LOG_MM_REQUEST, "MMU_:   Requested load from v.", hex_u<64>, req.mref->vaddr,
            " with ", req.mref->size, " bytes will throw. Exception set.");
        
        *req.exception = setExcept(ex_PF,
            present | (rx == MM::p_x ? expf_ifetch : 0) | (state.ring == pl_user ? expf_user : 0));
        
        req.mref->ready = MM::mr_valready;
        return 1;
    }

    // no dependency check needed! core laod/store check will handle this
    req.mref->ready = MM::mr_inexec;
    util::log(LOG_MM_REQUEST, "MMU_:   Load from v.", hex_u<64>, req.mref->vaddr, " requested. Expected latency ",
        MM_LD_LATENCY, " cycles.");
    
    req.cycle = state.cycle + MM_LD_LATENCY;
    ldbuf.push_back(req);
    return 0;
}

// add a store request to the store queue
u8 MemoryManager::put(MM::MemoryRequest& req)
{
    u8 present = !!(pagetable.count(pageFloor(req.mref->vaddr)) &&
        pagetable.count(pageFloor(req.mref->vaddr + req.mref->size - 1)));

    if(!present || bad_pl(req.mref->vaddr, req.mref->size) || bad_rwx(req.mref->vaddr, req.mref->size, MM::p_w))
    {
        util::log(LOG_MM_REQUEST, "MMU_:   Requested store to v.", hex_u<64>, req.mref->vaddr,
            " with ", req.mref->size, " bytes will throw. Exception status set.");
        
        *req.exception = setExcept(ex_PF, present | expf_write | (state.ring == pl_user ? expf_user : 0));
        req.mref->ready = MM::mr_valready;
        return 1;
    }
    // this request is not allowed to throw or fault in any way, the core views this store as commited
    util::log(LOG_MM_REQUEST, "MMU_:   Store to v.", hex_u<64>, req.mref->vaddr, " requested. Expected latency ",
        MM_ST_LATENCY, " cycles.");

    req.cycle = state.cycle + MM_ST_LATENCY;
    
    void* localbuf = aligned_alloc(req.mref->size, req.mref->size);
    if(!localbuf) throw AllocationFailedException(); // needed for correct values after many cycles, preg may be invalid
    else
    {
        std::memcpy(localbuf, req.mref->data, req.mref->size);
        req.mref->data = localbuf;
    }

    MM::StoreRequest sreq = { *req.mref, req.cycle };
    stbuf.push_back(sreq);
    return 0;
}

// read from vaddr into data, return latency and actual number of bytes read
pair<u64, u64> MemoryManager::read(u64 vaddr, void* data, size_t len, u8 rx)
{
    util::log(LOG_MM_EXEC, "MMU_:   Trying to read ", dec_u<0>, len, " bytes from v.",
        hex_u<64>, vaddr, ".");

    if((vaddr > VADDR_LIMIT)) throw InvalidPageaddrException();
    if(!pagetable.count(pageFloor(vaddr))) throw PageNotMappedException();

    u64 latency    = 0;
    u64 bytes_read = 0;
    try
    {
        if((pageOffs(vaddr) + len) > mem.at(pageFloor(get_paddr(vaddr, rx))).bytes_used)
        {
            util::log(LOG_MM_EXEC, "MMU_:   Read across bounds detected.");

            size_t pagebytes = mem.at(pageFloor(get_paddr(vaddr, rx))).bytes_used - pageOffs(vaddr);
            size_t rem_bytes = len; // total bytes remaining
            size_t offset    = 0;   // offset from vaddr

            // // test memory before actually reading anything
            // get_eaddr(vaddr, rx);
            // get_eaddr(vaddr + rem_bytes - 1, rx);

            for(;rem_bytes > 0;)
            {
                std::memcpy((void*)((uptr)data + offset), get_eaddr(vaddr + offset, rx), pagebytes);
                bytes_read += pagebytes;

                // we can't read any further if this is a partial page! successive vaddrs are invalid
                if(mem.at(pageFloor(get_paddr(vaddr + offset, rx))).bytes_used < PAGE_SIZE) [[unlikely]]
                {
                    util::log(LOG_MM_EXEC, "MMU_:   End of mapped region reached!");
                    break;
                }

                offset     += pagebytes;
                rem_bytes  -= pagebytes;
                pagebytes  =  (rem_bytes <= PAGE_SIZE) ? rem_bytes : PAGE_SIZE;
            }
        }
        else std::memcpy(data, get_eaddr(vaddr, rx), len);

        latency = MM_LD_LATENCY;
    } // rethrow any exceptions caused by address lookups
    catch(const MemoryManagerException& mme) { throw; }

    // util::log(0, "MMU_:   Read successful, latency ", dec_u<0>, latency, " cycles.");
    util::log(LOG_MM_EXEC, "MMU_:   Read ", dec_u<0>, (bytes_read ? bytes_read : len), " bytes.\n");

    return std::make_pair(latency, (bytes_read ? bytes_read : len));
}

void MemoryManager::write(u64 vaddr, void* data, size_t len)
{
    util::log(LOG_MM_EXEC, "MMU_:   Trying to write ", dec_u<0>, len, " bytes to v.",
        hex_u<64>, vaddr, ".");

    if((vaddr > VADDR_LIMIT)) throw InvalidPageaddrException();
    if(!pagetable.count(pageFloor(vaddr))) throw PageNotMappedException();

    try
    {
        if((pageOffs(vaddr) + len) > mem.at(pageFloor(get_paddr(vaddr, MM::p_w))).bytes_used)
        {
            util::log(LOG_MM_EXEC, "MMU_:   Write across bounds detected.");

            size_t pagebytes = mem.at(pageFloor(get_paddr(vaddr, MM::p_w))).bytes_used - pageOffs(vaddr);
            size_t rem_bytes = len; // total bytes remaining
            size_t offset    = 0;   // offset from vaddr

            // // test memory before actually writing anything
            // get_eaddr(vaddr, MM::p_w);
            // get_eaddr(vaddr + rem_bytes - 1, MM::p_w);

            for(;rem_bytes > 0;)
            {
                std::memcpy(get_eaddr(vaddr + offset, MM::p_w), (void*)((uptr)data + offset), pagebytes);

                // we can't write any further if this is a partial page! successive vaddrs are invalid
                if(mem.at(pageFloor(get_paddr(vaddr + offset, MM::p_w))).bytes_used < PAGE_SIZE) [[unlikely]]
                {
                    util::log(LOG_MM_EXEC, "MMU_:   End of mapped region reached!");
                    break;
                }

                offset    += pagebytes;
                rem_bytes -= pagebytes;
                pagebytes =  (rem_bytes <= PAGE_SIZE) ? rem_bytes : PAGE_SIZE;
            }
        }
        else std::memcpy(get_eaddr(vaddr, MM::p_w), data, len);
    } // rethrow any exceptions caused by address lookups
    catch(const MemoryManagerException& mme) { throw; }

    util::log(LOG_MM_EXEC, "MMU_:   Write successful.\n");
}

// void* malloc(size_t size) { return nullptr; }
// void free(void* ptr) { };
// void* calloc(size_t cnt, size_t size) { return memset(nullptr, 0, size*cnt); }
