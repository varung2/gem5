#ifndef __MEM_CACHE_PREFETCH_SANDBOX_HH__
#define __MEM_CACHE_PREFETCH_SANDBOX_HH__

#include "mem/physical.hh"
#include "mem/abstract_mem.hh"
#include "mem/cache/prefetch/queued.hh"
#include "mem/cache/base.hh"
#include "base/stats/types.hh"
// #include "sim/stats.hh" // used for estimating bw utilization info
#include "debug/HWPrefetch.hh"
#include <string>
#include <unordered_map>
#include <vector>
#include <deque>
#include <limits.h>
// #include "bf/all.hpp"
#include "base/filters/perfect_bloom_filter.hh"
#include "params/BloomFilterPerfect.hh"
#define PAGE_SIZE 4096
#define LOG2_PAGE_SIZE 12

// CACHE
#define BLOCK_SIZE 64
#define LOG2_BLOCK_SIZE 6

namespace gem5 {

struct SandboxPrefetcherParams; //done


GEM5_DEPRECATED_NAMESPACE(Prefetcher, prefetch);
namespace prefetch
{

class Score
{
  public:
    int32_t offset;
    uint32_t score;
    Score() : offset(0), score(0) {}
    Score(int32_t _offset) : offset(_offset), score(0) {}
    Score(int32_t _offset, uint32_t _score) : offset(_offset), score(_score) {}
    ~Score(){}
};

class Sandbox : public Queued
{
  private:
    std::deque<Score*> evaluated_offsets;
    std::deque<int32_t> non_evaluated_offsets;
    uint32_t pref_degree; /* degree per direction */
    struct
    {
      uint32_t curr_ptr;
      uint32_t total_demand;
      uint32_t filter_hit;
    } eval;
    gem5::BloomFilterPerfectParams q;

    /* Bloom Filter */
    uint32_t opt_hash_functions;
    bloom_filter::Base *bf;


  private:
    const uint32_t sandbox_pref_degree;
    const bool     sandbox_enable_stream_detect;
    const uint32_t sandbox_stream_detect_length;
    const uint32_t sandbox_num_access_in_phase;
    const uint32_t sandbox_num_cycle_offsets;
    const uint32_t sandbox_bloom_filter_size;
    const uint32_t sandbox_seed;
    void init_evaluated_offsets(); //done
    void init_non_evaluated_offsets(); //done
    void reset_eval();
    void get_offset_list_sorted(std::vector<Score*> &pos_offsets, std::vector<Score*> &neg_offsets); //done
    void generate_prefetch(std::vector<Score*> offset_list, uint32_t pref_degree, uint64_t page, uint32_t offset, std::vector<uint64_t> &pref_addr);//done
    void destroy_offset_list(std::vector<Score*> offset_list); //done
    uint64_t generate_address(uint64_t page, uint32_t offset, int32_t delta, uint32_t lookahead = 1); //done
    void end_of_round(); //done
    void filter_add(uint64_t address); //done
    bool filter_lookup(uint64_t address); //done

  public:
    Sandbox(const SandboxPrefetcherParams &p); //done
    ~Sandbox();
    void calculatePrefetch(const PrefetchInfo &pfi,
							   std::vector<AddrPriority> &addresses) override; //done

};

} // namespace prefetch
} // namespace gem5


#endif//__MEM_CACHE_PREFETCH_DSPATCH_HH__