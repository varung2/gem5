
#ifndef __MEM_CACHE_PREFETCH_PYTHIA_HH__
#define __MEM_CACHE_PREFETCH_PYTHIA_HH__

// gem5 deps
#include "mem/cache/base.hh"
#include "mem/mem_interface.hh"
#include "mem/cache/prefetch/queued.hh"

// pythia deps
#include "mem/cache/prefetch/scooby_helper.h"
#include "mem/cache/prefetch/learning_engine_basic.h"
#include "mem/cache/prefetch/learning_engine_featurewise.h"

// std lib deps
#include <vector>
#include <unordered_map>

#define TICKS_PER_SECOND 1e+12

// some pythia limits
#define MAX_ACTIONS 64
#define MAX_REWARDS 16
#define MAX_SCOOBY_DEGREE 16
#define SCOOBY_MAX_IPC_LEVEL 4
#define CACHE_ACC_LEVELS 10

// cache parameters - assume these are true
#define BLOCK_SIZE 64
#define LOG2_BLOCK_SIZE 6
#define PAGE_SIZE 4096
#define LOG2_PAGE_SIZE 12

namespace gem5 {

/* forward declaration */
class LearningEngine;

struct PythiaPrefetcherParams;

GEM5_DEPRECATED_NAMESPACE(Prefetcher, prefetch);
namespace prefetch {

// DSPatch prefetcher class
class Pythia : public Queued {
	private:
		std::deque<Scooby_STEntry*> signature_table;
		LearningEngineBasic *brain;
		LearningEngineFeaturewise *brain_featurewise;
		std::deque<Scooby_PTEntry*> prefetch_tracker;
		Scooby_PTEntry *last_evicted_tracker;
		uint8_t bw_level;
		uint8_t core_ipc;
		uint32_t acc_level;

		/* for workload insights only
		 * has nothing to do with prefetching */
		ScoobyRecorder *recorder;

		std::unordered_map<uint32_t, std::vector<uint64_t> > state_action_dist;
		std::unordered_map<std::string, std::vector<uint64_t> > state_action_dist2;
		std::unordered_map<int32_t, std::vector<uint64_t> > action_deg_dist;

		/* Data structures for debugging */
		std::unordered_map<std::string, uint64_t> target_action_state;

		// bw calculations
		std::vector<double> last_num_bytes_consumed;
		std::vector<Tick> last_tick;

	private:
		void init_knobs();
		uint8_t calculateBWBucket();
		uint32_t calculateAccuracyBucket();
		void update_global_state(uint64_t pc, uint64_t page, uint32_t offset, uint64_t address);
		Scooby_STEntry* update_local_state(uint64_t pc, uint64_t page, uint32_t offset, uint64_t address);
		uint32_t predict(uint64_t address, uint64_t page, uint32_t offset, State *state, vector<uint64_t> &pref_addr);
		bool track(uint64_t address, State *state, uint32_t action_index, Scooby_PTEntry **tracker);
		void reward(uint64_t address);
		void reward(Scooby_PTEntry *ptentry);
		void assign_reward(Scooby_PTEntry *ptentry, RewardType type);
		int32_t compute_reward(Scooby_PTEntry *ptentry, RewardType type);
		void train(Scooby_PTEntry *curr_evicted, Scooby_PTEntry *last_evicted);
		vector<Scooby_PTEntry*> search_pt(uint64_t address, bool search_all = false);
		void update_stats(uint32_t state, uint32_t action_index, uint32_t pref_degree = 1);
		void update_stats(State *state, uint32_t action_index, uint32_t degree = 1);
		void track_in_st(uint64_t page, uint32_t pred_offset, int32_t pref_offset);
		void gen_multi_degree_pref(uint64_t page, uint32_t offset, int32_t action, uint32_t pref_degree, vector<uint64_t> &pref_addr);
		uint32_t get_dyn_pref_degree(float max_to_avg_q_ratio, uint64_t page = 0xdeadbeef, int32_t action = 0); /* only implemented for CMAC engine 2.0 */
		bool is_high_bw();

	public:
		Pythia(const PythiaPrefetcherParams &p);
		~Pythia();

		void calculatePrefetch(const PrefetchInfo &pfi,
							   std::vector<AddrPriority> &addresses) override;

		void register_fill(uint64_t address);
		void register_prefetch_hit(uint64_t address);
		int32_t getAction(uint32_t action_index);
		void update_bw(uint8_t bw_level);
		// void update_ipc(uint8_t ipc);
		// void update_acc(uint32_t acc_level);
};

} // end namespace prefetch
} // end namespace gem5

#endif