
#ifndef __MEM_CACHE_PREFETCH_DSPATCH_HH__
#define __MEM_CACHE_PREFETCH_DSPATCH_HH__

// #include "mem/physical.hh"
#include "mem/mem_interface.hh"
#include "mem/cache/prefetch/queued.hh"
#include "mem/cache/base.hh"
#include "base/stats/types.hh"
// #include "sim/stats.hh" // used for estimating bw utilization info
#include "debug/HWPrefetch.hh"

#include <vector>
#include <deque>
#include <limits.h>

// cache parameters - assume these are true
#define BLOCK_SIZE 64
#define LOG2_BLOCK_SIZE 6

namespace gem5 {

// class BaseCache;

struct DSPatchPrefetcherParams;

// -------------------------------------------------------------------------------------
#ifndef BITMAP_H
	#define BITMAP_H
	#include <bitset>
	#include <string>
	#define BITMAP_MAX_SIZE 64
	typedef std::bitset<BITMAP_MAX_SIZE> Bitmap;
	class BitmapHelper{
	public:
		static uint64_t value(Bitmap bmp, uint32_t size = BITMAP_MAX_SIZE);
		static std::string to_string(Bitmap bmp, uint32_t size = BITMAP_MAX_SIZE);
		static uint32_t count_bits_set(Bitmap bmp, uint32_t size = BITMAP_MAX_SIZE);
		static uint32_t count_bits_same(Bitmap bmp1, Bitmap bmp2, uint32_t size = BITMAP_MAX_SIZE);
		static uint32_t count_bits_diff(Bitmap bmp1, Bitmap bmp2, uint32_t size = BITMAP_MAX_SIZE);
		static Bitmap rotate_left(Bitmap bmp, uint32_t amount, uint32_t size = BITMAP_MAX_SIZE);
		static Bitmap rotate_right(Bitmap bmp, uint32_t amount, uint32_t size = BITMAP_MAX_SIZE);
		static Bitmap compress(Bitmap bmp, uint32_t granularity, uint32_t size = BITMAP_MAX_SIZE);
		static Bitmap decompress(Bitmap bmp, uint32_t granularity, uint32_t size = BITMAP_MAX_SIZE);
		static Bitmap bitwise_or(Bitmap bmp1, Bitmap bmp2, uint32_t size = BITMAP_MAX_SIZE);
		static Bitmap bitwise_and(Bitmap bmp1, Bitmap bmp2, uint32_t size = BITMAP_MAX_SIZE);
	};
#endif /* BITMAP_H */

#ifndef DSPATCH_UTIL_H
	#define DSPATCH_UTIL_H
	#include <cstdint>
	#include <string>
	#include <sstream>

	void gen_random(char *s, const int len);
	uint32_t folded_xor(uint64_t value, uint32_t num_folds);

	template <class T> std::string array_to_string(std::vector<T> array, bool hex = false, uint32_t size = 0) {
	    std::stringstream ss;
	    if (size == 0) size = array.size();
	    for(uint32_t index = 0; index < size; ++index) {
	    	if(hex) {
	    		ss << std::hex << array[index] << std::dec;
	    	}
	    	else {
	    		ss << array[index];
	    	}
	        ss << ",";
	    }
	    return ss.str();
	}

	class HashZoo {
	public:
		static uint32_t jenkins(uint32_t key);
		static uint32_t knuth(uint32_t key);
		static uint32_t murmur3(uint32_t key);
		static uint32_t jenkins32(uint32_t key);
		static uint32_t hash32shift(uint32_t key);
		static uint32_t hash32shiftmult(uint32_t key);
		static uint32_t hash64shift(uint32_t key);
		static uint32_t hash5shift(uint32_t key);
		static uint32_t hash7shift(uint32_t key);
		static uint32_t Wang6shift(uint32_t key);
		static uint32_t Wang5shift(uint32_t key);
		static uint32_t Wang4shift( uint32_t key);
		static uint32_t Wang3shift( uint32_t key);

	    static uint32_t three_hybrid1(uint32_t key);
	    static uint32_t three_hybrid2(uint32_t key);
	    static uint32_t three_hybrid3(uint32_t key);
	    static uint32_t three_hybrid4(uint32_t key);
	    static uint32_t three_hybrid5(uint32_t key);
	    static uint32_t three_hybrid6(uint32_t key);
	    static uint32_t three_hybrid7(uint32_t key);
	    static uint32_t three_hybrid8(uint32_t key);
	    static uint32_t three_hybrid9(uint32_t key);
	    static uint32_t three_hybrid10(uint32_t key);
	    static uint32_t three_hybrid11(uint32_t key);
	    static uint32_t three_hybrid12(uint32_t key);

	    static uint32_t four_hybrid1(uint32_t key);
	    static uint32_t four_hybrid2(uint32_t key);
	    static uint32_t four_hybrid3(uint32_t key);
	    static uint32_t four_hybrid4(uint32_t key);
	    static uint32_t four_hybrid5(uint32_t key);
	    static uint32_t four_hybrid6(uint32_t key);
	    static uint32_t four_hybrid7(uint32_t key);
	    static uint32_t four_hybrid8(uint32_t key);
	    static uint32_t four_hybrid9(uint32_t key);
	    static uint32_t four_hybrid10(uint32_t key);
	    static uint32_t four_hybrid11(uint32_t key);
	    static uint32_t four_hybrid12(uint32_t key);

	    static uint32_t getHash(uint32_t selector, uint32_t key);
	};
#endif /* DSPATCH_UTIL_H */
// -------------------------------------------------------------------------------------

GEM5_DEPRECATED_NAMESPACE(Prefetcher, prefetch);
namespace prefetch {

// setting up structs
#define DSPATCH_MAX_BW_LEVEL 4
enum DSPatch_pref_candidate {
	NONE = 0,
	COVP,
	ACCP,
	Num_DSPatch_pref_candidates
};
const char* Map_DSPatch_pref_candidate(DSPatch_pref_candidate candidate);

class DSPatch_counter {
	private:
		uint32_t counter;
	public:
		DSPatch_counter() : counter(0) {}
		~DSPatch_counter(){}
		inline void incr(uint32_t max = UINT_MAX) {counter = (counter < max ? counter+1 : counter);}
		inline void decr(uint32_t min = 0) {counter = (counter > min ? counter-1 : counter);}
		inline uint32_t value(){return counter;}
		inline void reset(){counter = 0;}
};
// class describing the Page Buffer (PB) entry
class DSPatch_PBEntry {
	public:
		uint64_t page;
		uint64_t trigger_pc;
		uint32_t trigger_offset;
		Bitmap bmp_real;

		DSPatch_PBEntry() : page(0xdeadbeef), trigger_pc(0xdeadbeef), trigger_offset(0) {
			bmp_real.reset();
		}
		~DSPatch_PBEntry(){}
};
// class describing the Signature Prediction Table (SPT) entry
class DSPatch_SPTEntry {
	public:
		uint64_t signature;
		Bitmap bmp_cov;
		Bitmap bmp_acc;
		DSPatch_counter measure_covP, measure_accP;
		DSPatch_counter or_count;
		/* TODO: add confidence counters */

		DSPatch_SPTEntry() : signature(0xdeadbeef) {
			bmp_cov.reset();
			bmp_acc.reset();
			measure_covP.reset();
			measure_accP.reset();
			or_count.reset();
		}
		~DSPatch_SPTEntry(){}
};

// DSPatch prefetcher class
class DSPatch : public Queued {
	private:
		std::deque<DSPatch_PBEntry*> page_buffer;
		DSPatch_SPTEntry **spt;
		std::deque<Addr> pref_buffer;

		// vars for calculating BW
		const uint64_t dram__max_calculated_bw;
		double last_total_bytes_consumed;
		double last_sim_seconds;
		double mem_percentage_bw_used;

		/* 0 => b/w is less than 25% of peak
		 * 1 => b/w is more than 25% and less than 50% of peak
		 * 2 => b/w is more than 50% and less than 75% of peak
		 * 3 => b/w is more than 75% of peak
		 */
		uint8_t bw_bucket; 

		// parameters
		const uint32_t dspatch_log2_region_size;
		const uint32_t dspatch_num_cachelines_in_region;
		const uint32_t dspatch_pb_size;
		const uint32_t dspatch_num_spt_entries;
		const uint32_t dspatch_compression_granularity;
		const uint32_t dspatch_pred_throttle_bw_thr;
		const uint32_t dspatch_bitmap_selection_policy;
		const uint32_t dspatch_sig_type;
		const uint32_t dspatch_sig_hash_type;
		const uint32_t dspatch_or_count_max;
		const uint32_t dspatch_measure_covP_max;
		const uint32_t dspatch_measure_accP_max;
		const uint32_t dspatch_acc_thr;
		const uint32_t dspatch_cov_thr;
		const uint32_t dspatch_pref_buffer_size;
		const uint32_t dspatch_pref_degree;
		const bool     dspatch_enable_pref_buffer;

	private:
		DSPatch_pref_candidate select_bitmap(DSPatch_SPTEntry *sptentry, Bitmap &bmp_selected);
		DSPatch_PBEntry* search_pb(uint64_t page);
		
		// not sure what these do...
		void buffer_prefetch(std::vector<uint64_t> pref_addr);
		void issue_prefetch(std::vector<uint64_t> &pref_addr);

		uint32_t get_spt_index(uint64_t signature);
		uint32_t get_hash(uint32_t key);
		void add_to_spt(DSPatch_PBEntry *pbentry);
		DSPatch_pref_candidate dyn_selection(DSPatch_SPTEntry *sptentry, Bitmap &bmp_selected);
		void generate_prefetch(uint64_t pc, uint64_t page, uint32_t offset, uint64_t address, std::vector<uint64_t> &pref_addr);

		uint8_t calculateBWBucket();

	public:
		DSPatch(const DSPatchPrefetcherParams &p);
		~DSPatch();

		void calculatePrefetch(const PrefetchInfo &pfi,
							   std::vector<AddrPriority> &addresses) override;
};


} // namespace prefetch
} // namespace gem5


#endif//__MEM_CACHE_PREFETCH_DSPATCH_HH__


// OLD CODE
	// inside DSPatch::getMemoryBandwidth()
	// statistics::VResult rvec;
	// // sys->getPhysMem().memories[pmem_idx].stats.bwTotal.result(rvec);
	// simSeconds.result(rvec);
	// statistics::VResult rvec;
	// double bytesRead = sys->getPhysMem().memories[pmem_idx]->stats.bytesRead.total();
	// double bytesWritten = sys->getPhysMem().memories[pmem_idx]->stats.bytesWritten.total();
	// last_sim_seconds = simSeconds; // -> gives error 
	/* build/ARM/mem/cache/prefetch/dspatch.cc:428:23: error: cannot convert 'gem5::statistics::Formula' to 'double' in assignment
		  428 |    last_sim_seconds = simSeconds;
		      |                       ^~~~~~~~~~
		      |                       |
		      |                       gem5::statistics::Formula 
	*/


	// for (size_t ridx = 0; ridx < rvec.size(); ridx++) {
	// 	// double bw_val = rvec[ridx].getValue();
	// 	double bw_val = rvec[ridx];
	// 	std::cout << "DEBUG: pmem_idx = " << pmem_idx << " | ridx = " << ridx << " | bw_val = " << bw_val << std::endl;
	// 	bus_utilization += bw_val;
	// }
	// std::cout << "DEBUG: pmem_idx = " << pmem_idx << " | bytesRead = " << bytesRead << " | bytesWritten = " << bytesWritten << std::endl;