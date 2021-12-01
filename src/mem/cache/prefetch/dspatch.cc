#include "mem/cache/prefetch/dspatch.hh"
#include "params/DSPatchPrefetcher.hh"

#include <assert.h>
#include <sstream>
#include <algorithm>
#include <iomanip>
#include <cassert>
#include <iostream>

namespace gem5 {

// BITMAP Helper class
	std::string BitmapHelper::to_string(Bitmap bmp, uint32_t size) {
		// return bmp.to_string<char,std::string::traits_type,std::string::allocator_type>();
		std::stringstream ss;
		for(int32_t bit = size-1; bit >= 0; --bit) {
			ss << bmp[bit];
		}
		return ss.str();
	}
	uint32_t BitmapHelper::count_bits_set(Bitmap bmp, uint32_t size) {
		// return static_cast<uint32_t>(bmp.count());
		uint32_t count = 0;
		for(uint32_t index = 0; index < size; ++index) {
			if(bmp[index]) count++;
		}
		return count;
	}
	uint32_t BitmapHelper::count_bits_same(Bitmap bmp1, Bitmap bmp2, uint32_t size) {
		uint32_t count_same = 0;
		for(uint32_t index = 0; index < size; ++index) {
			if(bmp1[index] && bmp1[index] == bmp2[index]) count_same++;
		}
		return count_same;
	}
	uint32_t BitmapHelper::count_bits_diff(Bitmap bmp1, Bitmap bmp2, uint32_t size) {
		uint32_t count_diff = 0;
		for(uint32_t index = 0; index < size; ++index) {
			if(bmp1[index] && !bmp2[index]) count_diff++;
		}
		return count_diff;
	}
	uint64_t BitmapHelper::value(Bitmap bmp, uint32_t size){
		return bmp.to_ullong();
	}
	Bitmap BitmapHelper::rotate_left(Bitmap bmp, uint32_t amount, uint32_t size) {
		Bitmap result;
		for(uint32_t index = 0; index < (size - amount); ++index)
			result[index+amount] = bmp[index];
		for(uint32_t index = 0; index < amount; ++index)
			result[index] = bmp[index+size-amount];
		return result;
	}
	Bitmap BitmapHelper::rotate_right(Bitmap bmp, uint32_t amount, uint32_t size) {
		Bitmap result;
		for(uint32_t index = 0; index < size - amount; ++index)
			result[index] = bmp[index+amount];
		for(uint32_t index = 0; index < amount; ++index)
			result[size-amount+index] = bmp[index];
		return result;
	}
	Bitmap BitmapHelper::compress(Bitmap bmp, uint32_t granularity, uint32_t size) {
		assert(size % granularity == 0);
		uint32_t index = 0;
		Bitmap result;
		uint32_t ptr = 0;

		while(index < size) {
			bool res = false;
			uint32_t gran = 0;
			for(gran = 0; gran < granularity; ++gran) {
				assert(index + gran < size);
				res = res | bmp[index+gran];
			}
			result[ptr] = res;
			ptr++;
			index = index + gran;
		}
		return result;
	}
	Bitmap BitmapHelper::decompress(Bitmap bmp, uint32_t granularity, uint32_t size) {
		Bitmap result;
		result.reset();
		assert(size*granularity <= BITMAP_MAX_SIZE);
		for(uint32_t index = 0; index < size; ++index) {
			if(bmp[index]) {
				uint32_t ptr = index * granularity;
				for(uint32_t count = 0; count < granularity; ++count) result[ptr+count] = true;
			}
		}
		return result;
	}
	Bitmap BitmapHelper::bitwise_or(Bitmap bmp1, Bitmap bmp2, uint32_t size) {
		Bitmap result;
		for(uint32_t index = 0; index < size; ++index) {
			if(bmp1[index] || bmp2[index])
				result[index] = true;
		}
		return result;
	}
	Bitmap BitmapHelper::bitwise_and(Bitmap bmp1, Bitmap bmp2, uint32_t size) {
		Bitmap result;
		for(uint32_t index = 0; index < size; ++index) {
			if(bmp1[index] && bmp2[index]) result[index] = true;
		}
		return result;
	}
// END BITMAP Helper class

// DSPATCHUTIL Functions
	/* helper function */
	void gen_random(char *s, const int len) 
	{
	    static const char alphanum[] =
	        "0123456789"
	        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
	        "abcdefghijklmnopqrstuvwxyz";

	    for (int i = 0; i < len; ++i) 
	    {
	        s[i] = alphanum[rand() % (sizeof(alphanum) - 1)];
	    }
	    
	    s[len] = 0;
	}

	uint32_t folded_xor(uint64_t value, uint32_t num_folds)
	{
		assert(num_folds > 1);
		assert((num_folds & (num_folds-1)) == 0); /* has to be power of 2 */
		uint32_t mask = 0;
		uint32_t bits_in_fold = 64/num_folds;
		if(num_folds == 2)
		{
			mask = 0xffffffff;
		}
		else
		{
			mask = (1ul << bits_in_fold) - 1;
		}
		uint32_t folded_value = 0;
		for(uint32_t fold = 0; fold < num_folds; ++fold)
		{
			folded_value = folded_value ^ ((value >> (fold * bits_in_fold)) & mask);
		}
		return folded_value;
	}

	uint32_t HashZoo::jenkins(uint32_t key)
	{
	    // Robert Jenkins' 32 bit mix function
	    key += (key << 12);
	    key ^= (key >> 22);
	    key += (key << 4);
	    key ^= (key >> 9);
	    key += (key << 10);
	    key ^= (key >> 2);
	    key += (key << 7);
	    key ^= (key >> 12);
	    return key;
	}

	uint32_t HashZoo::knuth(uint32_t key)
	{
	    // Knuth's multiplicative method
	    key = (key >> 3) * 2654435761;
	    return key;
	}

	uint32_t HashZoo::murmur3(uint32_t key)
	{
		/* TODO: define it using murmur3's finilization steps */
		assert(false);
	}

	/* originally ment for 32b key */
	uint32_t HashZoo::jenkins32(uint32_t key)
	{
	   key = (key+0x7ed55d16) + (key<<12);
	   key = (key^0xc761c23c) ^ (key>>19);
	   key = (key+0x165667b1) + (key<<5);
	   key = (key+0xd3a2646c) ^ (key<<9);
	   key = (key+0xfd7046c5) + (key<<3);
	   key = (key^0xb55a4f09) ^ (key>>16);
	   return key;
	}

	/* originally ment for 32b key */
	uint32_t HashZoo::hash32shift(uint32_t key)
	{
		key = ~key + (key << 15); // key = (key << 15) - key - 1;
		key = key ^ (key >> 12);
		key = key + (key << 2);
		key = key ^ (key >> 4);
		key = key * 2057; // key = (key + (key << 3)) + (key << 11);
		key = key ^ (key >> 16);
		return key;
	}

	/* originally ment for 32b key */
	uint32_t HashZoo::hash32shiftmult(uint32_t key)
	{
		int c2=0x27d4eb2d; // a prime or an odd constant
		key = (key ^ 61) ^ (key >> 16);
		key = key + (key << 3);
		key = key ^ (key >> 4);
		key = key * c2;
		key = key ^ (key >> 15);
		return key;
	}

	uint32_t HashZoo::hash64shift(uint32_t key)
	{
		key = (~key) + (key << 21); // key = (key << 21) - key - 1;
		key = key ^ (key >> 24);
		key = (key + (key << 3)) + (key << 8); // key * 265
		key = key ^ (key >> 14);
		key = (key + (key << 2)) + (key << 4); // key * 21
		key = key ^ (key >> 28);
		key = key + (key << 31);
		return key;
	}

	uint32_t HashZoo::hash5shift(uint32_t key)
	{
		key = (key ^ 61) ^ (key >> 16);
	    key = key + (key << 3);
	    key = key ^ (key >> 4);
	    key = key * 0x27d4eb2d;
	    key = key ^ (key >> 15);
	    return key;
	}

	/* hash6shift is jenkin32 */

	uint32_t HashZoo::hash7shift(uint32_t key)
	{
	    key -= (key << 6);
	    key ^= (key >> 17);
	    key -= (key << 9);
	    key ^= (key << 4);
	    key -= (key << 3);
	    key ^= (key << 10);
	    key ^= (key >> 15);
	    return key ;
	}

	/* use low bit values */
	uint32_t HashZoo::Wang6shift(uint32_t key)
	{
	    key += ~(key << 15);
	    key ^=  (key >> 10);
	    key +=  (key << 3);
	    key ^=  (key >> 6);
	    key += ~(key << 11);
	    key ^=  (key >> 16);
	    return key;
	}

	uint32_t HashZoo::Wang5shift(uint32_t key)
	{
	    key = (key + 0x479ab41d) + (key << 8);
	    key = (key ^ 0xe4aa10ce) ^ (key >> 5);
	    key = (key + 0x9942f0a6) - (key << 14);
	    key = (key ^ 0x5aedd67d) ^ (key >> 3);
	    key = (key + 0x17bea992) + (key << 7);
	    return key;
	}

	uint32_t HashZoo::Wang4shift( uint32_t key)
	{
	    key = (key ^ 0xdeadbeef) + (key << 4);
	    key = key ^ (key >> 10);
	    key = key + (key << 7);
	    key = key ^ (key >> 13);
	    return key;
	}

	uint32_t HashZoo::Wang3shift( uint32_t key)
	{
	    key = key ^ (key >> 4);
	    key = (key ^ 0xdeadbeef) + (key << 5);
	    key = key ^ (key >> 11);
	    return key;
	}

	uint32_t HashZoo::three_hybrid1(uint32_t key) { return knuth(hash64shift(jenkins32(key))); }
	uint32_t HashZoo::three_hybrid2(uint32_t key) { return jenkins32(Wang5shift(hash5shift(key))); }
	uint32_t HashZoo::three_hybrid3(uint32_t key) { return jenkins(hash32shiftmult(Wang3shift(key))); }
	uint32_t HashZoo::three_hybrid4(uint32_t key) { return Wang6shift(hash32shift(Wang5shift(key))); }
	uint32_t HashZoo::three_hybrid5(uint32_t key) { return hash64shift(hash32shift(knuth(key))); }
	uint32_t HashZoo::three_hybrid6(uint32_t key) { return hash5shift(jenkins(Wang6shift(key))); }
	uint32_t HashZoo::three_hybrid7(uint32_t key) { return Wang4shift(jenkins32(hash7shift(key))); }
	uint32_t HashZoo::three_hybrid8(uint32_t key) { return Wang3shift(Wang6shift(hash64shift(key))); }
	uint32_t HashZoo::three_hybrid9(uint32_t key) { return hash32shift(Wang3shift(jenkins(key))); }
	uint32_t HashZoo::three_hybrid10(uint32_t key) { return hash32shiftmult(Wang4shift(hash32shiftmult(key))); }
	uint32_t HashZoo::three_hybrid11(uint32_t key) { return hash7shift(hash5shift(Wang4shift(key))); }
	uint32_t HashZoo::three_hybrid12(uint32_t key) { return Wang5shift(jenkins32(hash32shift(key))); }

	uint32_t HashZoo::four_hybrid1(uint32_t key) { return Wang6shift(Wang5shift(Wang3shift(Wang4shift(key)))); }
	uint32_t HashZoo::four_hybrid2(uint32_t key) { return hash32shiftmult(jenkins(Wang5shift(Wang6shift(key)))); }
	uint32_t HashZoo::four_hybrid3(uint32_t key) { return hash64shift(hash7shift(jenkins32(hash32shift(key)))); }
	uint32_t HashZoo::four_hybrid4(uint32_t key) { return knuth(knuth(hash32shiftmult(hash5shift(key)))); }
	uint32_t HashZoo::four_hybrid5(uint32_t key) { return jenkins32(Wang4shift(hash64shift(hash32shiftmult(key)))); }
	uint32_t HashZoo::four_hybrid6(uint32_t key) { return jenkins(hash32shift(Wang4shift(Wang3shift(key)))); }
	uint32_t HashZoo::four_hybrid7(uint32_t key) { return hash32shift(hash64shift(hash5shift(hash64shift(key)))); }
	uint32_t HashZoo::four_hybrid8(uint32_t key) { return hash7shift(hash5shift(hash32shiftmult(Wang6shift(key)))); }
	uint32_t HashZoo::four_hybrid9(uint32_t key) { return hash32shiftmult(Wang6shift(jenkins32(knuth(key)))); }
	uint32_t HashZoo::four_hybrid10(uint32_t key) { return Wang3shift(jenkins32(knuth(jenkins(key)))); }
	uint32_t HashZoo::four_hybrid11(uint32_t key) { return hash5shift(hash32shiftmult(hash32shift(jenkins32(key)))); }
	uint32_t HashZoo::four_hybrid12(uint32_t key) { return Wang4shift(Wang3shift(jenkins(hash7shift(key)))); }

	uint32_t HashZoo::getHash(uint32_t selector, uint32_t key)
	{
	    switch(selector)
	    {
	        case 1:     return key;
	        case 2:     return jenkins(key);
	        case 3:     return knuth(key);
	        case 4:     return murmur3(key);
	        case 5:     return jenkins32(key);
	        case 6:     return hash32shift(key);
	        case 7:     return hash32shiftmult(key);
	        case 8:     return hash64shift(key);
	        case 9:     return hash5shift(key);
	        case 10:    return hash7shift(key);
	        case 11:    return Wang6shift(key);
	        case 12:    return Wang5shift(key);
	        case 13:    return Wang4shift(key);
	        case 14:    return Wang3shift(key);
	        
	        /* three hybrid */
	        case 101:  return three_hybrid1(key);
	        case 102:  return three_hybrid2(key);
	        case 103:  return three_hybrid3(key);
	        case 104:  return three_hybrid4(key);
	        case 105:  return three_hybrid5(key);
	        case 106:  return three_hybrid6(key);
	        case 107:  return three_hybrid7(key);
	        case 108:  return three_hybrid8(key);
	        case 109:  return three_hybrid9(key);
	        case 110:  return three_hybrid10(key);
	        case 111:  return three_hybrid11(key);
	        case 112:  return three_hybrid12(key);

	        /* four hybrid */
	        case 1001:  return four_hybrid1(key);
	        case 1002:  return four_hybrid2(key);
	        case 1003:  return four_hybrid3(key);
	        case 1004:  return four_hybrid4(key);
	        case 1005:  return four_hybrid5(key);
	        case 1006:  return four_hybrid6(key);
	        case 1007:  return four_hybrid7(key);
	        case 1008:  return four_hybrid8(key);
	        case 1009:  return four_hybrid9(key);
	        case 1010:  return four_hybrid10(key);
	        case 1011:  return four_hybrid11(key);
	        case 1012:  return four_hybrid12(key);

	        default:    assert(false);
	    }
	}
// END DSPATCHUTIL Functions

GEM5_DEPRECATED_NAMESPACE(Prefetcher, prefetch);
namespace prefetch {

	const char* DSPatch_pref_candidate_string[] = {"NONE", "CovP", "AccP"};
	const char* Map_DSPatch_pref_candidate(DSPatch_pref_candidate candidate) {
		assert((uint32_t)candidate < DSPatch_pref_candidate::Num_DSPatch_pref_candidates);
		return DSPatch_pref_candidate_string[(uint32_t)candidate];
	}

	DSPatch::DSPatch(const DSPatchPrefetcherParams &p) :
		Queued(p),
		dspatch_log2_region_size(p.log2_region_size),
		dspatch_num_cachelines_in_region(p.num_cachelines_in_region),
		dspatch_pb_size(p.pb_size),
		dspatch_num_spt_entries(p.num_spt_entries),
		dspatch_compression_granularity(p.compression_granularity),
		dspatch_pred_throttle_bw_thr(p.pred_throttle_bw_thr),
		dspatch_bitmap_selection_policy(p.bitmap_selection_policy),
		dspatch_sig_type(p.sig_type),
		dspatch_sig_hash_type(p.sig_hash_type),
		dspatch_or_count_max(p.or_count_max),
		dspatch_measure_covP_max(p.measure_covP_max),
		dspatch_measure_accP_max(p.measure_accP_max),
		dspatch_acc_thr(p.acc_thr),
		dspatch_cov_thr(p.cov_thr),
		dspatch_enable_pref_buffer(p.enable_pref_buffer),
		dspatch_pref_buffer_size(p.pref_buffer_size),
		dspatch_pref_degree(p.pref_degree),
		// dram__freq(2400),
		// dram__bus_width(4*16),
		// 2400MHz (clks/sec) * 2 transfers/clk (DDR) * buswidth (bits) / 8 bits * # interfaces (i.e., dual-channel)
		// = Max throughput in Bytes/second 
		dram__max_calculated_bw(2400000000*2* ((4*16) >> 3) * 2) 
		{ 

		assert(dspatch_log2_region_size <= 12);
		assert(dspatch_num_cachelines_in_region * dspatch_compression_granularity <= 64);

		bw_bucket = 0;
		spt = (DSPatch_SPTEntry**)calloc(dspatch_num_spt_entries, sizeof(DSPatch_SPTEntry**));
		assert(spt);
		for(uint32_t index = 0; index < dspatch_num_spt_entries; ++index) {
			spt[index] = new DSPatch_SPTEntry();
		}
	}

	DSPatch::~DSPatch() {
		for(uint32_t index = 0; index < dspatch_num_spt_entries; ++index) {
			delete spt[index];
		}
		free(spt);
	}

	uint8_t DSPatch::getMemoryBandwidth() {
		System* sys = cache->system;
		double total_bw = 0;
		for (size_t pmem_idx = 0; pmem_idx < sys->getPhysMem().memories.size(); pmem_idx++) {
			statistics::VResult rvec;
			sys->getPhysMem().memories[pmem_idx]->stats.bwTotal.result(rvec);
			for (size_t ridx = 0; ridx < rvec.size(); ridx++) {
				// double bw_val = rvec[ridx].getValue();
				double bw_val = rvec[ridx];
				std::cout << "DEBUG: pmem_idx = " << pmem_idx << " | ridx = " << ridx << " | bw_val = " << bw_val << std::endl;
				total_bw += bw_val;
			}
		}
		std::cout << "DEBUG: finished calculating total bw consumed => " << total_bw << std::endl;
		return bw_bucket;
	}

	void DSPatch::calculatePrefetch(const PrefetchInfo &pfi, std::vector<AddrPriority> &addresses) {
		Addr address = pfi.getAddr();
		uint64_t page = address >> dspatch_log2_region_size;
		uint32_t offset = (address >> LOG2_BLOCK_SIZE) & ((1ull << (dspatch_log2_region_size - LOG2_BLOCK_SIZE)) - 1);

		if (!pfi.hasPC()) return; // if pc is not valid, return from calculate prefetch
		Addr pc = pfi.getPC();

		bw_bucket = getMemoryBandwidth();

		// search page buffer for page
		DSPatch_PBEntry *pbentry = NULL;
		pbentry = search_pb(page);
		if (pbentry) {
			/* record the access */
			pbentry->bmp_real[offset] = true;
		} else { /* page buffer miss, prefetch trigger opportunity */
			/* insert the new page buffer entry */
			if(page_buffer.size() >= dspatch_pb_size) {
				pbentry = page_buffer.front();
				page_buffer.pop_front();
				add_to_spt(pbentry);
				delete pbentry;
			}

			pbentry = new DSPatch_PBEntry();
			pbentry->page = page;
			pbentry->trigger_pc = pc;
			pbentry->trigger_offset = offset;
			pbentry->bmp_real[offset] = true;
			page_buffer.push_back(pbentry);
		}

		std::vector<uint64_t> pref_addr;
		/* trigger prefetch */
		generate_prefetch(pc, page, offset, address, pref_addr);

		// after generating prefetch addresses - load w/ priority into vector
		for (size_t i = 0; i < pref_addr.size(); i++) {
			Addr address = Addr(pref_addr[i]);
			addresses.push_back(AddrPriority(address, i));
		}

	}


	DSPatch_pref_candidate DSPatch::select_bitmap(DSPatch_SPTEntry *sptentry, Bitmap &bmp_selected) {
		DSPatch_pref_candidate candidate = DSPatch_pref_candidate::NONE;
		switch(dspatch_bitmap_selection_policy) {
			case 1:
				/* always select coverage bitmap */
				bmp_selected = sptentry->bmp_cov;
				candidate = DSPatch_pref_candidate::COVP;
				break;
			case 2:
				/* always select accuracy bitmap */
				bmp_selected = sptentry->bmp_acc;
				candidate = DSPatch_pref_candidate::ACCP;
				break;
			case 3:
				/* hybrid selection */
				candidate = dyn_selection(sptentry, bmp_selected);
				break;
			default:
				std::cout << "invalid dspatch_bitmap_selection_policy " << dspatch_bitmap_selection_policy << std::endl;
				assert(false);
		}
		return candidate;
	}
	DSPatch_PBEntry* DSPatch::search_pb(uint64_t page) {
		auto it = find_if(page_buffer.begin(), page_buffer.end(), [page](DSPatch_PBEntry *pbentry){return pbentry->page == page;});
		return it != page_buffer.end() ? (*it) : NULL;
	}
	void DSPatch::buffer_prefetch(std::vector<uint64_t> pref_addr) {
		uint32_t count = 0;
		for(uint32_t index = 0; index < pref_addr.size(); ++index) {
			if(pref_buffer.size() >= dspatch_pref_buffer_size)
				break;
			pref_buffer.push_back(pref_addr[index]);
			count++;
		}
	}
	void DSPatch::issue_prefetch(std::vector<uint64_t> &pref_addr) {
		uint32_t count = 0;
		while(!pref_buffer.empty() && count < dspatch_pref_degree) {
			pref_addr.push_back(pref_buffer.front());
			pref_buffer.pop_front();
			count++;
		}
	}
	uint32_t DSPatch::get_spt_index(uint64_t signature) {
		uint32_t folded_sig = folded_xor(signature, 2);
		/* apply hash */
		uint32_t hashed_index = get_hash(folded_sig);
		return hashed_index % dspatch_num_spt_entries;
	}
	uint32_t DSPatch::get_hash(uint32_t key) {
		switch(dspatch_sig_hash_type) {
			case 1: 	return key;
			case 2: 	return HashZoo::jenkins(key);
			case 3: 	return HashZoo::knuth(key);
			case 4: 	return HashZoo::murmur3(key);
			case 5: 	return HashZoo::jenkins32(key);
			case 6: 	return HashZoo::hash32shift(key);
			case 7: 	return HashZoo::hash32shiftmult(key);
			case 8: 	return HashZoo::hash64shift(key);
			case 9: 	return HashZoo::hash5shift(key);
			case 10: 	return HashZoo::hash7shift(key);
			case 11: 	return HashZoo::Wang6shift(key);
			case 12: 	return HashZoo::Wang5shift(key);
			case 13: 	return HashZoo::Wang4shift(key);
			case 14: 	return HashZoo::Wang3shift(key);
			default: 	assert(false);
		}
	}
	void DSPatch::add_to_spt(DSPatch_PBEntry *pbentry) {
		Bitmap bmp_real, bmp_cov, bmp_acc;
		bmp_real = pbentry->bmp_real;
		uint64_t trigger_pc = pbentry->trigger_pc;
		uint32_t trigger_offset = pbentry->trigger_offset;

		uint64_t signature = trigger_pc;
		uint32_t spt_index = get_spt_index(signature);
		assert(spt_index < dspatch_num_spt_entries);
		DSPatch_SPTEntry *sptentry = spt[spt_index];

		bmp_real = BitmapHelper::rotate_right(bmp_real, trigger_offset, dspatch_num_cachelines_in_region);
		bmp_cov  = BitmapHelper::decompress(sptentry->bmp_cov, dspatch_compression_granularity, dspatch_num_cachelines_in_region);
		bmp_acc  = BitmapHelper::decompress(sptentry->bmp_acc, dspatch_compression_granularity, dspatch_num_cachelines_in_region);

		uint32_t pop_count_bmp_real = BitmapHelper::count_bits_set(bmp_real);
		uint32_t pop_count_bmp_cov  = BitmapHelper::count_bits_set(bmp_cov);
		uint32_t pop_count_bmp_acc  = BitmapHelper::count_bits_set(bmp_acc);
		uint32_t same_count_bmp_cov = BitmapHelper::count_bits_same(bmp_cov, bmp_real);
		uint32_t same_count_bmp_acc = BitmapHelper::count_bits_same(bmp_acc, bmp_real);

		uint32_t cov_bmp_cov = 100 * (float)same_count_bmp_cov / pop_count_bmp_real;
		uint32_t acc_bmp_cov = 100 * (float)same_count_bmp_cov / pop_count_bmp_cov;
		uint32_t cov_bmp_acc = 100 * (float)same_count_bmp_acc / pop_count_bmp_real;
		uint32_t acc_bmp_acc = 100 * (float)same_count_bmp_acc / pop_count_bmp_acc;

		/* Update CovP counters */
		if(BitmapHelper::count_bits_diff(bmp_real, bmp_cov) != 0)
			sptentry->or_count.incr(dspatch_or_count_max);
		if(acc_bmp_cov < dspatch_acc_thr || cov_bmp_cov < dspatch_cov_thr)
			sptentry->measure_covP.incr(dspatch_measure_covP_max);

		/* Update CovP */
		if(sptentry->measure_covP.value() == dspatch_measure_covP_max) {
			if(bw_bucket == 3 || cov_bmp_cov < 50) /* WARNING: hardcoded values */ {
				sptentry->bmp_cov = BitmapHelper::compress(bmp_real, dspatch_compression_granularity);
				sptentry->or_count.reset();
			}
		} else {
			sptentry->bmp_cov = BitmapHelper::compress(BitmapHelper::bitwise_or(bmp_cov, bmp_real), dspatch_compression_granularity);
		}

		/* Update AccP counter(s) */
		if(acc_bmp_acc < 50) /* WARNING: hardcoded value */	{
			sptentry->measure_accP.incr();
		}
		else {
			sptentry->measure_accP.decr();
		}

		/* Update AccP */
		sptentry->bmp_acc = BitmapHelper::bitwise_and(bmp_real, BitmapHelper::decompress(sptentry->bmp_cov, dspatch_compression_granularity, dspatch_num_cachelines_in_region));
		sptentry->bmp_acc = BitmapHelper::compress(sptentry->bmp_acc, dspatch_compression_granularity);
	}
	DSPatch_pref_candidate DSPatch::dyn_selection(DSPatch_SPTEntry *sptentry, Bitmap &bmp_selected) {
		DSPatch_pref_candidate candidate = DSPatch_pref_candidate::NONE;

		if (bw_bucket == 3) {
			if (sptentry->measure_accP.value() == dspatch_measure_accP_max) {
				/* no prefetch */
				bmp_selected.reset();
				candidate = DSPatch_pref_candidate::NONE;
			}
			else {
				/* Prefetch with accP */
				bmp_selected = sptentry->bmp_acc;
				candidate = DSPatch_pref_candidate::ACCP;
			}
		}
		else if (bw_bucket == 2) {
			if (sptentry->measure_covP.value() == dspatch_measure_covP_max) {
				/* Prefetch with accP */
				bmp_selected = sptentry->bmp_acc;
				candidate = DSPatch_pref_candidate::ACCP;
			}
			else {
				/* Prefetch with covP */
				bmp_selected = sptentry->bmp_cov;
				candidate = DSPatch_pref_candidate::COVP;
			}
		}
		else {
			/* Prefetch with covP */
			bmp_selected = sptentry->bmp_cov;
			candidate = DSPatch_pref_candidate::COVP;		
		}
		return candidate;
	}
	void DSPatch::generate_prefetch(uint64_t pc, uint64_t page, uint32_t offset, uint64_t address, std::vector<uint64_t> &pref_addr) {
		Bitmap bmp_cov, bmp_acc, bmp_pred;
		uint64_t signature = 0xdeadbeef;
		DSPatch_pref_candidate candidate = DSPatch_pref_candidate::NONE;
		DSPatch_SPTEntry *sptentry = NULL;

		signature = pc;
		uint32_t spt_index = get_spt_index(signature);
		assert(spt_index < dspatch_num_spt_entries);

		sptentry = spt[spt_index];
		candidate = select_bitmap(sptentry, bmp_pred);

		/* decompress and rotate back the bitmap */
		bmp_pred = BitmapHelper::decompress(bmp_pred, dspatch_compression_granularity, dspatch_num_cachelines_in_region);
		bmp_pred = BitmapHelper::rotate_left(bmp_pred, offset, dspatch_num_cachelines_in_region);

		/* Throttling predictions incase of predicting with bmp_acc and b/w is high */
		if(bw_bucket >= dspatch_pred_throttle_bw_thr && candidate == DSPatch_pref_candidate::ACCP) {
			bmp_pred.reset();
		}

		for(uint32_t index = 0; index < dspatch_num_cachelines_in_region; ++index) {
			if(bmp_pred[index] && index != offset) {
				uint64_t addr = (page << dspatch_log2_region_size) + (index << LOG2_BLOCK_SIZE);
				pref_addr.push_back(addr);
			}
		}
	}

} // end namespace prefetch
} // end namespace gem5

