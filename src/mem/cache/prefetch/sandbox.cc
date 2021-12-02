
#include "mem/cache/prefetch/sandbox.hh"
#include "params/SandboxPrefetcher.hh"


#include <assert.h>
#include <sstream>
#include <algorithm>
#include <iomanip>
#include <cassert>
#include <iostream>

namespace gem5 {

GEM5_DEPRECATED_NAMESPACE(Prefetcher, prefetch);
namespace prefetch {


Sandbox::Sandbox(const SandboxPrefetcherParams &p):
        Queued(p),
        sandbox_pref_degree(p.sandbox_pref_degree),
        sandbox_enable_stream_detect(p.sandbox_enable_stream_detect),
        sandbox_stream_detect_length(p.sandbox_stream_detect_length),
        sandbox_num_access_in_phase(p.sandbox_num_access_in_phase),
        sandbox_num_cycle_offsets(p.sandbox_num_cycle_offsets),
        sandbox_bloom_filter_size(p.sandbox_bloom_filter_size),
        sandbox_seed(p.sandbox_seed)
        {
		q.size = 1;
		q.num_bits= 1;
		q.threshold = 1;

		opt_hash_functions = (uint32_t)ceil(sandbox_bloom_filter_size / sandbox_num_access_in_phase * log(2));
			bf = new bloom_filter::Perfect(q);
		assert(bf);

		init_evaluated_offsets();
		init_non_evaluated_offsets();
		reset_eval();

}

Sandbox::~Sandbox() {
		delete bf;
}

void Sandbox::init_evaluated_offsets()
        {
        /* select {-8,-1} and {+1,+8} offsets in the beginning */
        for(int32_t index = 1; index <= 8; ++index){
                Score *score = new Score(index);
                evaluated_offsets.push_back(score);
        }
        for(int32_t index = -8; index <= -1; ++index){
                Score *score = new Score(index);
                evaluated_offsets.push_back(score);
        }
}

void Sandbox::init_non_evaluated_offsets()
{
        for(int32_t index = 9; index <= 16; ++index){
                non_evaluated_offsets.push_back(index);
        }
        for(int32_t index = -16; index <= -9; ++index){
                non_evaluated_offsets.push_back(index);
        }
}

bool sort_function(Score *score1, Score *score2)
{
	return (abs(score1->offset) < abs(score2->offset));
}

void Sandbox::get_offset_list_sorted(std::vector<Score*> &pos_offsets, std::vector<Score*> &neg_offsets)
{
	for(uint32_t index = 0; index < evaluated_offsets.size(); ++index){
		assert(evaluated_offsets[index]->offset != 0);
		if(evaluated_offsets[index]->offset > 0){
			pos_offsets.push_back(new Score(evaluated_offsets[index]->offset, evaluated_offsets[index]->score));
		}
		else{
			neg_offsets.push_back(new Score(evaluated_offsets[index]->offset, evaluated_offsets[index]->score));			
		}
	}

	std::sort(pos_offsets.begin(), pos_offsets.end(), [](Score *score1, Score *score2){return abs(score1->offset) < abs(score2->offset);});
	std::sort(neg_offsets.begin(), neg_offsets.end(), [](Score *score1, Score *score2){return abs(score1->offset) < abs(score2->offset);});
}


void Sandbox::destroy_offset_list(std::vector<Score*> offset_list)
{
	for(uint32_t index = 0; index < offset_list.size(); ++index)
	{
		delete offset_list[index];
	}
}

uint64_t Sandbox::generate_address(uint64_t page, uint32_t offset, int32_t delta, uint32_t lookahead)
{
	int32_t pref_offset = offset + delta * lookahead;
	if(pref_offset >= 0 && pref_offset < 64)
	{
		return ((page << LOG2_PAGE_SIZE) + (pref_offset << LOG2_BLOCK_SIZE));
	}
	else
	{
		return 0xdeadbeef;
	}
}

void Sandbox::end_of_round()
{
	/* sort evaluated offset list based on score */
	std::sort(evaluated_offsets.begin(), evaluated_offsets.end(), [](Score *score1, Score *score2){return score1->score > score2->score;});

	/* cycle-out n lowest performing offsets */
	for(uint32_t count = 0; count < sandbox_num_cycle_offsets; ++count){
		if(evaluated_offsets.empty())
		{
			break;
		}
		Score *score = evaluated_offsets.back();
		evaluated_offsets.pop_back();
		non_evaluated_offsets.push_back(score->offset);
		delete score;
	}

	/* cycle-in next n non_evaluated_offsets */
	for(uint32_t count = 0; count < sandbox_num_cycle_offsets; ++count){
		int32_t offset = non_evaluated_offsets.front();
		non_evaluated_offsets.pop_front();
		Score *score = new Score(offset);
		evaluated_offsets.push_back(score);
	}
}

void Sandbox::filter_add(uint64_t address)
{
	bf->set(address);
}

bool Sandbox::filter_lookup(uint64_t address)
{
	return bf->getCount(address);
}

void Sandbox::reset_eval()
{
	eval.curr_ptr = 0;
	eval.total_demand = 0;
	eval.filter_hit = 0;
	bf->clear();
}



void Sandbox::calculatePrefetch(const PrefetchInfo &pfi, std::vector<AddrPriority> &addresses)
{
	Addr address = pfi.getAddr();
	uint64_t page = address >> LOG2_PAGE_SIZE;
	uint32_t offset = (address >> LOG2_BLOCK_SIZE) & ((1ull << (LOG2_PAGE_SIZE - LOG2_BLOCK_SIZE)) - 1);

	assert(evaluated_offsets.size() == 16);
	assert(eval.curr_ptr >= 0 && eval.curr_ptr < evaluated_offsets.size());

	// stats.called++;

	/* ======================  Four step process  ======================
	 * Step 1: check demand access in bloom filter to manipulate score.
	 * Step 2: generate pseudo prefetch request and add to bloom filter.
	 * Step 3: check the end of current phase or the end of current round. 
	 * Step 4: generate actual prefetch reuqests based on the scores. */

	/* Step 1: check demand access in bloom filter to manipulate score */
	eval.total_demand++;
	// stats.step1.filter_lookup++;
	bool lookup = filter_lookup(address);
	if(lookup)
	{
		//stats.step1.filter_hit++;
		eval.filter_hit++;
		/* increment score */
		evaluated_offsets[eval.curr_ptr]->score++;
		if(sandbox_enable_stream_detect)
		{
			/* RBERA: TODO */
			for(uint32_t index = 1; index <= sandbox_stream_detect_length; ++index)
			{
				int32_t stream_offset = offset - (evaluated_offsets[eval.curr_ptr]->offset * index);
				if(stream_offset >= 0 && stream_offset < 64)
				{
					uint64_t stream_addr = (page << LOG2_PAGE_SIZE) + (stream_offset << LOG2_BLOCK_SIZE);
					if(filter_lookup(stream_addr))
					{
						evaluated_offsets[eval.curr_ptr]->score++;
					}
				}
			}
		}
	}

	/* Step 2: generate pseudo prefetch request and add to bloom filter */
	uint32_t pref_offset = offset + evaluated_offsets[eval.curr_ptr]->offset;
	if(pref_offset >= 0 && pref_offset < 64)
	{
		uint64_t pseudo_pref_addr = (page << LOG2_PAGE_SIZE) + (pref_offset << LOG2_BLOCK_SIZE);
		filter_add(pseudo_pref_addr);
		// stats.step2.filter_add++;
	}

	/* Step 3: check the end of current phase or the end of current round */
	/* Phase end := the current offset has been evaluated for a number of cache accesses
	 * Round end := all 16 offsets has been evaluated */
	if(eval.total_demand == sandbox_num_access_in_phase)
	{
		// stats.step3.end_of_phase++;
		uint32_t next_ptr = eval.curr_ptr + 1;
		if(next_ptr < evaluated_offsets.size())
		{
			reset_eval();
			eval.curr_ptr = next_ptr;
		}
		else
		{
			/* end of current round */
			// stats.step3.end_of_round++;
			end_of_round();
			reset_eval();
			eval.curr_ptr = 0;
		}
	}

	/* Step 4: generate actual prefetch reuqests based on the scores */
	std::vector<Score*> pos_offsets, neg_offsets;
	/* Following function does two job:
	 * 1. Creates two separate lists of Scores based on posetive and negative offsets
	 * 2. Sorts both offset lists based on ABSOLUTE offset value, as Snadbox prefers smaller offsets for prefetching */
	get_offset_list_sorted(pos_offsets, neg_offsets);
	std::vector<uint64_t> pref_addr;
	/* generate prefetches */
	generate_prefetch(pos_offsets, sandbox_pref_degree, page, offset, pref_addr);
	uint32_t pos_pref = pref_addr.size();
	generate_prefetch(neg_offsets, sandbox_pref_degree, page, offset, pref_addr);
	uint32_t neg_pref = pref_addr.size() - pos_pref;
	/* destroy temporary lists */
	destroy_offset_list(pos_offsets);
	destroy_offset_list(neg_offsets);
	for (size_t i = 0; i < pref_addr.size(); i++) {
			Addr address = Addr(pref_addr[i]);
			addresses.push_back(AddrPriority(address, i));
	}
	// stats.step4.pref_generated += pref_addr.size();
	// stats.step4.pref_generated_pos += pos_pref;
	// stats.step4.pref_generated_neg += neg_pref;
}


void Sandbox::generate_prefetch(std::vector<Score*> offset_list, uint32_t pref_degree, uint64_t page, uint32_t offset, std::vector<uint64_t> &pref_addr)
{
	uint32_t count = 0;
	for(uint32_t index = 0; index < offset_list.size(); ++index)
	{
		for(uint32_t lookahead = 1; lookahead <= sandbox_stream_detect_length+1; ++lookahead)
		{
			if(lookahead > 1 && !sandbox_enable_stream_detect)
			{
				break;
			}

			if(offset_list[index]->score >= lookahead*sandbox_num_access_in_phase)
			{
				uint64_t addr = generate_address(page, offset, offset_list[index]->offset, lookahead);
				if(addr != 0xdeadbeef)
				{
					pref_addr.push_back(addr);
					count++;
					// record_pref_stats(offset_list[index]->offset, 1);
				}
			}
		}
		
		if(count > pref_degree)
		{
			break;
		}
	}
}

} // end namespace prefetch
} // end namespace gem5