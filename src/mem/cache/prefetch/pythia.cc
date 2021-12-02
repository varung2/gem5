#include "mem/cache/prefetch/pythia.hh"
#include "params/PythiaPrefetcher.hh"

/* Action array
 * Basically a set of deltas to evaluate
 * Similar to the concept of BOP */
std::vector<int32_t> Actions;

namespace knob {
	extern float    scooby_alpha;
	extern float    scooby_gamma;
	extern float    scooby_epsilon;
	extern uint32_t scooby_state_num_bits;
	extern uint32_t scooby_max_states;
	extern uint32_t scooby_seed;
	extern string   scooby_policy;
	extern string   scooby_learning_type;
	extern vector<int32_t> scooby_actions;
	extern uint32_t scooby_max_actions;
	extern uint32_t scooby_pt_size;
	extern uint32_t scooby_st_size;
	extern uint32_t scooby_max_pcs;
	extern uint32_t scooby_max_offsets;
	extern uint32_t scooby_max_deltas;
	extern int32_t  scooby_reward_none;
	extern int32_t  scooby_reward_incorrect;
	extern int32_t  scooby_reward_correct_untimely;
	extern int32_t  scooby_reward_correct_timely;
	extern bool		scooby_brain_zero_init;
	extern bool     scooby_enable_reward_all;
	extern bool     scooby_enable_track_multiple;
	extern bool     scooby_enable_reward_out_of_bounds;
	extern int32_t  scooby_reward_out_of_bounds;
	extern uint32_t scooby_state_type;
	extern bool     scooby_access_debug;
	extern bool     scooby_print_access_debug;
	extern uint64_t scooby_print_access_debug_pc;
	extern uint32_t scooby_print_access_debug_pc_count;
	extern bool     scooby_print_trace;
	extern bool     scooby_enable_state_action_stats;
	extern bool     scooby_enable_reward_tracker_hit;
	extern int32_t  scooby_reward_tracker_hit;
	extern uint32_t scooby_state_hash_type;
	extern bool     scooby_enable_featurewise_engine;
	extern uint32_t scooby_pref_degree;
	extern bool     scooby_enable_dyn_degree;
	extern vector<float> scooby_max_to_avg_q_thresholds;
	extern vector<int32_t> scooby_dyn_degrees;
	extern uint64_t scooby_early_exploration_window;
	extern uint32_t scooby_pt_address_hash_type;
	extern uint32_t scooby_pt_address_hashed_bits;
	extern uint32_t scooby_bloom_filter_size;
	extern uint32_t scooby_multi_deg_select_type;
	extern vector<int32_t> scooby_last_pref_offset_conf_thresholds;
	extern vector<int32_t> scooby_dyn_degrees_type2;
	extern uint32_t scooby_action_tracker_size;
	extern uint32_t scooby_high_bw_thresh;
	extern bool     scooby_enable_hbw_reward;
	extern int32_t  scooby_reward_hbw_correct_timely;
	extern int32_t  scooby_reward_hbw_correct_untimely;
	extern int32_t  scooby_reward_hbw_incorrect;
	extern int32_t  scooby_reward_hbw_none;
	extern int32_t  scooby_reward_hbw_out_of_bounds;
	extern int32_t  scooby_reward_hbw_tracker_hit;
	extern vector<int32_t> scooby_last_pref_offset_conf_thresholds_hbw;
	extern vector<int32_t> scooby_dyn_degrees_type2_hbw;

	/* Learning Engine knobs */
	extern bool     le_enable_trace;
	extern uint32_t le_trace_interval;
	extern std::string   le_trace_file_name;
	extern uint32_t le_trace_state;
	extern bool     le_enable_score_plot;
	extern std::vector<int32_t> le_plot_actions;
	extern std::string   le_plot_file_name;
	extern bool     le_enable_action_trace;
	extern uint32_t le_action_trace_interval;
	extern std::string le_action_trace_name;
	extern bool     le_enable_action_plot;

	/* Featurewise Engine knobs */
	extern vector<int32_t> 	le_featurewise_active_features;
	extern vector<int32_t> 	le_featurewise_num_tilings;
	extern vector<int32_t> 	le_featurewise_num_tiles;
	extern vector<int32_t> 	le_featurewise_hash_types;
	extern vector<int32_t> 	le_featurewise_enable_tiling_offset;
	extern float			le_featurewise_max_q_thresh;
	extern bool				le_featurewise_enable_action_fallback;
	extern vector<float> 	le_featurewise_feature_weights;
	extern bool				le_featurewise_enable_dynamic_weight;
	extern float			le_featurewise_weight_gradient;
	extern bool				le_featurewise_disable_adjust_weight_all_features_align;
	extern bool				le_featurewise_selective_update;
	extern uint32_t 		le_featurewise_pooling_type;
	extern bool             le_featurewise_enable_dyn_action_fallback;
	extern uint32_t 		le_featurewise_bw_acc_check_level;
	extern uint32_t 		le_featurewise_acc_thresh;
	extern bool 			le_featurewise_enable_trace;
	extern uint32_t		le_featurewise_trace_feature_type;
	extern string 			le_featurewise_trace_feature;
	extern uint32_t 		le_featurewise_trace_interval;
	extern uint32_t 		le_featurewise_trace_record_count;
	extern std::string 	le_featurewise_trace_file_name;
	extern bool 			le_featurewise_enable_score_plot;
	extern vector<int32_t> le_featurewise_plot_actions;
	extern std::string 	le_featurewise_plot_file_name;
	extern bool 			le_featurewise_remove_plot_script;
}

namespace gem5 {

GEM5_DEPRECATED_NAMESPACE(Prefetcher, prefetch);
namespace prefetch {

// Public functions
	Pythia::Pythia(const PythiaPrefetcherParams &p) :
		Queued(p),
		last_num_bytes_consumed(std::vector<double>(1, 0)),
		last_tick(std::vector<Tick>(1, 0)) {
		init_knobs();
		recorder = new ScoobyRecorder();

		last_evicted_tracker = NULL;
		/* init learning engine */
		brain_featurewise = NULL;
		brain = NULL;

		if(knob::scooby_enable_featurewise_engine) {
			brain_featurewise = new LearningEngineFeaturewise(this,
															knob::scooby_alpha, knob::scooby_gamma, knob::scooby_epsilon,
															knob::scooby_max_actions,
															knob::scooby_seed,
															knob::scooby_policy,
															knob::scooby_learning_type,
															knob::scooby_brain_zero_init);
		}
		else {
			brain = new LearningEngineBasic(this,
										knob::scooby_alpha, knob::scooby_gamma, knob::scooby_epsilon,
										knob::scooby_max_actions,
										knob::scooby_max_states,
										knob::scooby_seed,
										knob::scooby_policy,
										knob::scooby_learning_type,
										knob::scooby_brain_zero_init,
										knob::scooby_early_exploration_window);
		}
		bw_level = 0;
		core_ipc = 0;
	}
	Pythia::~Pythia() {
		if (brain_featurewise) 	delete brain_featurewise;
		if (brain) 				delete brain;
	}
	void Pythia::calculatePrefetch(const PrefetchInfo &pfi, std::vector<AddrPriority> &addresses) {
		Addr address = pfi.getAddr();
		uint64_t page = address >> LOG2_PAGE_SIZE;
		uint32_t offset = (address >> LOG2_BLOCK_SIZE) & ((1ull << (LOG2_PAGE_SIZE - LOG2_BLOCK_SIZE)) - 1);

		if (!pfi.hasPC()) return; // if pc is not valid, return from calculate prefetch
		Addr pc = pfi.getPC();

		bw_level = calculateBWBucket();

		/* compute reward on demand */
		reward(uint64_t(address));

		/* record the access: just to gain some insights from the workload
		 * defined in scooby_helper.h(cc) */
		recorder->record_access(pc, address, page, offset, bw_level);

		/* global state tracking */
		update_global_state(pc, page, offset, address);
		/* per page state tracking */
		Scooby_STEntry *stentry = update_local_state(pc, page, offset, address);

		acc_level = calculateAccuracyBucket();

		/* Measure state.
		 * state can contain per page local information like delta signature, pc signature etc.
		 * it can also contain global signatures like last three branch PCs etc.
		 */
		State *state = new State();
		state->pc = pc;
		state->address = address;
		state->page = page;
		state->offset = offset;
		state->delta = !stentry->deltas.empty() ? stentry->deltas.back() : 0;
		state->local_delta_sig = stentry->get_delta_sig();
		state->local_delta_sig2 = stentry->get_delta_sig2();
		state->local_pc_sig = stentry->get_pc_sig();
		state->local_offset_sig = stentry->get_offset_sig();
		state->bw_level = bw_level;
		state->is_high_bw = is_high_bw();
		state->acc_level = acc_level;

		std::vector<uint64_t> pref_addr;

		// gen prefetches
		predict(address, page, offset, state, pref_addr);

		// after generating prefetch addresses - load w/ priority into vector
		for (size_t i = 0; i < pref_addr.size(); i++) {
			Addr address = Addr(pref_addr[i]);
			addresses.push_back(AddrPriority(address, i));
		}
	}

// Private functions
	void Pythia::init_knobs() {
		Actions.resize(knob::scooby_max_actions, 0);
		std::copy(knob::scooby_actions.begin(), knob::scooby_actions.end(), Actions.begin());
		assert(Actions.size() == knob::scooby_max_actions);
		assert(Actions.size() <= MAX_ACTIONS);
		if(knob::scooby_access_debug)
		{
			cout << "***WARNING*** setting knob::scooby_max_pcs, knob::scooby_max_offsets, and knob::scooby_max_deltas to large value as knob::scooby_access_debug is true" << endl;
			knob::scooby_max_pcs = 1024;
			knob::scooby_max_offsets = 1024;
			knob::scooby_max_deltas = 1024;
		}
		assert(knob::scooby_pref_degree >= 1 && (knob::scooby_pref_degree == 1 || !knob::scooby_enable_dyn_degree));
		assert(knob::scooby_max_to_avg_q_thresholds.size() == knob::scooby_dyn_degrees.size()-1);
		assert(knob::scooby_last_pref_offset_conf_thresholds.size() == knob::scooby_dyn_degrees_type2.size()-1);
	}
	uint32_t Pythia::calculateAccuracyBucket() {
		double pref_acc = prefetchStats.accuracy.total();
		uint32_t acc_lvl = (pref_acc / ((double)100/CACHE_ACC_LEVELS)); // quantize into 8 buckets
		if(acc_lvl >= CACHE_ACC_LEVELS) acc_lvl = (CACHE_ACC_LEVELS - 1);
		return acc_lvl;
	}
	uint8_t Pythia::calculateBWBucket() {
		System* sys = cache->system;
		double bus_utilization = 0;
		uint32_t dram_count = 0;

		// loop through memories
		for (size_t pmem_idx = 0; pmem_idx < sys->getPhysMem().memories.size(); pmem_idx++) {
			const memory::DRAMInterface* dram = dynamic_cast<memory::DRAMInterface*>(sys->getPhysMem().memories[pmem_idx]);

			// can use embedded stats from the DRAMInterface class
			if (dram) {
				// update vector sizes 
				// (update works like this because we assume that memory is not dynamically attached to the system during runtime)
				if (dram_count >= last_num_bytes_consumed.size())
					last_num_bytes_consumed.push_back(0);
				if (dram_count >= last_tick.size())
					last_tick.push_back(0);

				// calculate bw utilization using delta values
				double max_bw = dram->stats.peakBW.total();
				uint64_t bytes_consumed = (dram->stats.bytesRead.total() + dram->stats.bytesWritten.total());
				double delta_bytes = bytes_consumed - last_num_bytes_consumed[dram_count];
				double delta_t = double(curTick() - last_tick[dram_count]) / double(TICKS_PER_SECOND);
				bus_utilization += ((delta_bytes / delta_t) / max_bw);

				//update the last values
				last_tick[dram_count] = curTick();
				last_num_bytes_consumed[dram_count] = bytes_consumed;

				//inc dram count
				dram_count++;
			} 
			// memory type isn't of DRAMInterface... some other memory or device memory... don't factor in bw calculation
			// Note if using other types of memories, this code needs to be modified
		}
		bus_utilization /= dram_count;
		uint8_t calc_bw_bucket = uint8_t(bus_utilization/0.25);
		calc_bw_bucket = (calc_bw_bucket < 3) ? calc_bw_bucket : 3;
		
		// std::cout << "DEBUG: finished calculating avg bus utilization => " << bus_utilization << std::endl;
		// std::cout << "DEBUG: finished calculating bw bucket => " << calc_bw_bucket << std::endl;
		// return calc_bw_bucket;
		return 0; //hardcode to zero
	}
	void Pythia::update_global_state(uint64_t pc, uint64_t page, uint32_t offset, uint64_t address){}
	Scooby_STEntry* Pythia::update_local_state(uint64_t pc, uint64_t page, uint32_t offset, uint64_t address) {
		Scooby_STEntry *stentry = NULL;
		auto st_index = find_if(signature_table.begin(), signature_table.end(), [page](Scooby_STEntry *stentry){return stentry->page == page;});
		if(st_index != signature_table.end()) {
			stentry = (*st_index);
			stentry->update(page, pc, offset, address);
			signature_table.erase(st_index);
			signature_table.push_back(stentry);
			return stentry;
		}
		else {
			if(signature_table.size() >= knob::scooby_st_size)
			{
				stentry = signature_table.front();
				signature_table.pop_front();
				if(knob::scooby_access_debug)
				{
					recorder->record_access_knowledge(stentry);
					if(knob::scooby_print_access_debug)
					{
						print_access_debug(stentry);
					}
				}
				delete stentry;
			}

			stentry = new Scooby_STEntry(page, pc, offset);
			recorder->record_trigger_access(page, pc, offset);
			signature_table.push_back(stentry);
			return stentry;
		}
	}
	uint32_t Pythia::predict(uint64_t base_address, uint64_t page, uint32_t offset, State *state, std::vector<uint64_t> &pref_addr) {

		/* query learning engine to get the next prediction */
		uint32_t action_index = 0;
		uint32_t pref_degree = knob::scooby_pref_degree;
		std::vector<bool> consensus_vec; // only required for featurewise engine

		if (knob::scooby_enable_featurewise_engine) {
			float max_to_avg_q_ratio = 1.0;
			action_index = brain_featurewise->chooseAction(state, max_to_avg_q_ratio, consensus_vec);
			if(knob::scooby_enable_dyn_degree) {
				pref_degree = get_dyn_pref_degree(max_to_avg_q_ratio, page, Actions[action_index]);
			}
			if(knob::scooby_enable_state_action_stats) {
				update_stats(state, action_index, pref_degree);
			}
		}
		else {
			uint32_t state_index = state->value();
			assert(state_index < knob::scooby_max_states);
			action_index = brain->chooseAction(state_index);
			if(knob::scooby_enable_state_action_stats) {
				update_stats(state_index, action_index, pref_degree);
			}
		}
		assert(action_index < knob::scooby_max_actions);

		uint64_t addr = 0xdeadbeef;
		Scooby_PTEntry *ptentry = NULL;
		int32_t predicted_offset = 0;
		if(Actions[action_index] != 0) {
			predicted_offset = (int32_t)offset + Actions[action_index];
			if(predicted_offset >=0 && predicted_offset < 64) /* falls within the page */ {
				addr = (page << LOG2_PAGE_SIZE) + (predicted_offset << LOG2_BLOCK_SIZE);
				/* track prefetch */
				bool new_addr = track(addr, state, action_index, &ptentry);
				if(new_addr) {
					pref_addr.push_back(addr);
					track_in_st(page, predicted_offset, Actions[action_index]);
					if(pref_degree > 1)
					{
						gen_multi_degree_pref(page, offset, Actions[action_index], pref_degree, pref_addr);
					}
					ptentry->consensus_vec = consensus_vec;
				}
				else {
					if(knob::scooby_enable_reward_tracker_hit)
					{
						addr = 0xdeadbeef;
						track(addr, state, action_index, &ptentry);
						assert(ptentry);
						assign_reward(ptentry, RewardType::tracker_hit);
						ptentry->consensus_vec = consensus_vec;
					}
				}
			}
			else {
				if(knob::scooby_enable_reward_out_of_bounds) {
					addr = 0xdeadbeef;
					track(addr, state, action_index, &ptentry);
					assert(ptentry);
					assign_reward(ptentry, RewardType::out_of_bounds);
					ptentry->consensus_vec = consensus_vec;
				}
			}
		}
		else {
			/* agent decided not to prefetch */
			addr = 0xdeadbeef;
			/* track no prefetch */
			track(addr, state, action_index, &ptentry);
			ptentry->consensus_vec = consensus_vec;
		}
		return pref_addr.size();
	}
	bool Pythia::track(uint64_t address, State *state, uint32_t action_index, Scooby_PTEntry **tracker) {
		bool new_addr = true;
		std::vector<Scooby_PTEntry*> ptentries = search_pt(address, false);
		if(ptentries.empty()) {
			new_addr = true;
		}
		else {
			new_addr = false;
		}
		if(!new_addr && address != 0xdeadbeef && !knob::scooby_enable_track_multiple) {
			tracker = NULL;
			return new_addr;
		}

		/* new prefetched address that hasn't been seen before */
		Scooby_PTEntry *ptentry = NULL;

		if(prefetch_tracker.size() >= knob::scooby_pt_size) {
			ptentry = prefetch_tracker.front();
			prefetch_tracker.pop_front();
			if(last_evicted_tracker) {
				/* train the agent */
				train(ptentry, last_evicted_tracker);
				delete last_evicted_tracker->state;
				delete last_evicted_tracker;
			}
			last_evicted_tracker = ptentry;
		}

		ptentry = new Scooby_PTEntry(address, state, action_index);
		prefetch_tracker.push_back(ptentry);
		assert(prefetch_tracker.size() <= knob::scooby_pt_size);

		(*tracker) = ptentry;
		return new_addr;
	}
	void Pythia::gen_multi_degree_pref(uint64_t page, uint32_t offset, int32_t action, uint32_t pref_degree, std::vector<uint64_t> &pref_addr) {
		uint64_t addr = 0xdeadbeef;
		int32_t predicted_offset = 0;
		if(action != 0) {
			for(uint32_t degree = 2; degree <= pref_degree; ++degree) {
				predicted_offset = (int32_t)offset + degree * action;
				if(predicted_offset >=0 && predicted_offset < 64) {
					addr = (page << LOG2_PAGE_SIZE) + (predicted_offset << LOG2_BLOCK_SIZE);
					pref_addr.push_back(addr);
				}
			}
		}
	}
	uint32_t Pythia::get_dyn_pref_degree(float max_to_avg_q_ratio, uint64_t page, int32_t action) {
		uint32_t counted = false;
		uint32_t degree = 1;

		if(knob::scooby_multi_deg_select_type == 2) {
			auto st_index = find_if(signature_table.begin(), signature_table.end(), [page](Scooby_STEntry *stentry){return stentry->page == page;});
			if(st_index != signature_table.end()) {
				int32_t conf = 0;
				bool found = (*st_index)->search_action_tracker(action, conf);
				std::vector<int32_t> conf_thresholds, deg_afterburning, deg_normal;

				conf_thresholds = is_high_bw() ? knob::scooby_last_pref_offset_conf_thresholds_hbw : knob::scooby_last_pref_offset_conf_thresholds;
				deg_normal = is_high_bw() ? knob::scooby_dyn_degrees_type2_hbw : knob::scooby_dyn_degrees_type2;

				if(found) {
					for(uint32_t index = 0; index < conf_thresholds.size(); ++index) {
						/* scooby_last_pref_offset_conf_thresholds is a sorted list in ascending order of values */
						if(conf <= conf_thresholds[index]) {
							degree = deg_normal[index];
							counted = true;
							break;
						}
					}
					if(!counted) {
						degree = deg_normal.back();
					}
				}
				else {
					degree = 1;
				}
			}
		}
		return degree;
	}
	void Pythia::reward(uint64_t address) {
		std::vector<Scooby_PTEntry*> ptentries = search_pt(address, knob::scooby_enable_reward_all);
		if(ptentries.empty()) {
			// stats.reward.demand.pt_not_found++;
			return;
		}
		// else {
		// 	stats.reward.demand.pt_found++;
		// }

		for(uint32_t index = 0; index < ptentries.size(); ++index) {
			Scooby_PTEntry *ptentry = ptentries[index];
			/* Do not compute reward if already has a reward.
			 * This can happen when a prefetch access sees multiple demand reuse */
			if(ptentry->has_reward) {
				// MYLOG("entry already has reward: %d", ptentry->reward);
				// stats.reward.demand.has_reward++;
				return;
			}

			if(ptentry->is_filled) /* timely */ {
				assign_reward(ptentry, RewardType::correct_timely);
				// MYLOG("assigned reward correct_timely(%d)", ptentry->reward);
			}
			else {
				assign_reward(ptentry, RewardType::correct_untimely);
				// MYLOG("assigned reward correct_untimely(%d)", ptentry->reward);
			}
			ptentry->has_reward = true;
		}
	}
	void Pythia::reward(Scooby_PTEntry *ptentry) {
		// MYLOG("reward PT evict %lx state %x act_idx %u act %d", ptentry->address, ptentry->state->value(), ptentry->action_index, Actions[ptentry->action_index]);
		// stats.reward.train.called++;
		assert(!ptentry->has_reward);
		/* this is called during eviction from prefetch tracker
		 * that means, this address doesn't see a demand reuse.
		 * hence it either can be incorrect, or no prefetch */
		if(ptentry->address == 0xdeadbeef) /* no prefetch */ {
			assign_reward(ptentry, RewardType::none);
			// MYLOG("assigned reward no_pref(%d)", ptentry->reward);
		}
		else /* incorrect prefetch */
		{
			assign_reward(ptentry, RewardType::incorrect);
			// MYLOG("assigned reward incorrect(%d)", ptentry->reward);
		}
		ptentry->has_reward = true;
	}
	bool Pythia::is_high_bw(){return bw_level >= knob::scooby_high_bw_thresh ? true : false;
	}
	void Pythia::assign_reward(Scooby_PTEntry *ptentry, RewardType type) {
		// MYLOG("assign_reward PT evict %lx state %x act_idx %u act %d", ptentry->address, ptentry->state->value(), ptentry->action_index, Actions[ptentry->action_index]);
		assert(!ptentry->has_reward);

		/* compute the reward */
		int32_t reward = compute_reward(ptentry, type);

		/* assign */
		ptentry->reward = reward;
		ptentry->reward_type = type;
		ptentry->has_reward = true;

		/* maintain stats */
		// stats.reward.assign_reward.called++;
		// switch(type)
		// {
		// 	case RewardType::correct_timely: 	stats.reward.correct_timely++; break;
		// 	case RewardType::correct_untimely: 	stats.reward.correct_untimely++; break;
		// 	case RewardType::incorrect: 		stats.reward.incorrect++; break;
		// 	case RewardType::none: 				stats.reward.no_pref++; break;
		// 	case RewardType::out_of_bounds: 	stats.reward.out_of_bounds++; break;
		// 	case RewardType::tracker_hit: 		stats.reward.tracker_hit++; break;
		// 	default:							assert(false);
		// }
		// stats.reward.dist[ptentry->action_index][type]++;
	}
	int32_t Pythia::compute_reward(Scooby_PTEntry *ptentry, RewardType type) {
		bool high_bw = (knob::scooby_enable_hbw_reward && is_high_bw()) ? true : false;
		int32_t reward = 0;

		// stats.reward.compute_reward.dist[type][high_bw]++;

		if(type == RewardType::correct_timely)
		{
			reward = high_bw ? knob::scooby_reward_hbw_correct_timely : knob::scooby_reward_correct_timely;
		}
		else if(type == RewardType::correct_untimely)
		{
			reward = high_bw ? knob::scooby_reward_hbw_correct_untimely : knob::scooby_reward_correct_untimely;
		}
		else if(type == RewardType::incorrect)
		{
			reward = high_bw ? knob::scooby_reward_hbw_incorrect : knob::scooby_reward_incorrect;
		}
		else if(type == RewardType::none)
		{
			reward = high_bw ? knob::scooby_reward_hbw_none : knob::scooby_reward_none;
		}
		else if(type == RewardType::out_of_bounds)
		{
			reward = high_bw ? knob::scooby_reward_hbw_out_of_bounds : knob::scooby_reward_out_of_bounds;
		}
		else if(type == RewardType::tracker_hit)
		{
			reward = high_bw ? knob::scooby_reward_hbw_tracker_hit : knob::scooby_reward_tracker_hit;
		}
		else
		{
			cout << "Invalid reward type found " << type << endl;
			assert(false);
		}

		return reward;
	}
	void Pythia::train(Scooby_PTEntry *curr_evicted, Scooby_PTEntry *last_evicted) {
		// MYLOG("victim %s %u %d last_victim %s %u %d", curr_evicted->state->to_string().c_str(), curr_evicted->action_index, Actions[curr_evicted->action_index],
													// last_evicted->state->to_string().c_str(), last_evicted->action_index, Actions[last_evicted->action_index]);

		// stats.train.called++;
		if(!last_evicted->has_reward)
		{
			// stats.train.compute_reward++;
			reward(last_evicted);
		}
		assert(last_evicted->has_reward);

		/* train */
		// MYLOG("===SARSA=== S1: %s A1: %u R1: %d S2: %s A2: %u", last_evicted->state->to_string().c_str(), last_evicted->action_index,
																// last_evicted->reward,
																// curr_evicted->state->to_string().c_str(), curr_evicted->action_index);
		if(knob::scooby_enable_featurewise_engine)
		{
			brain_featurewise->learn(last_evicted->state, last_evicted->action_index, last_evicted->reward, curr_evicted->state, curr_evicted->action_index,
									last_evicted->consensus_vec, last_evicted->reward_type);
		}
		else
		{
			brain->learn(last_evicted->state->value(), last_evicted->action_index, last_evicted->reward, curr_evicted->state->value(), curr_evicted->action_index);
		}
		// MYLOG("train done");
	}
	void Pythia::register_fill(uint64_t address) {
		// MYLOG("fill @ %lx", address);

		// stats.register_fill.called++;
		std::vector<Scooby_PTEntry*> ptentries = search_pt(address, knob::scooby_enable_reward_all);
		if(!ptentries.empty())
		{
			// stats.register_fill.set++;
			for(uint32_t index = 0; index < ptentries.size(); ++index)
			{
				// stats.register_fill.set_total++;
				ptentries[index]->is_filled = true;
				// MYLOG("fill PT hit. pref with act_idx %u act %d", ptentries[index]->action_index, Actions[ptentries[index]->action_index]);
			}
		}
	}
	void Pythia::register_prefetch_hit(uint64_t address) {
		// MYLOG("pref_hit @ %lx", address);

		// stats.register_prefetch_hit.called++;
		std::vector<Scooby_PTEntry*> ptentries = search_pt(address, knob::scooby_enable_reward_all);
		if(!ptentries.empty())
		{
			// stats.register_prefetch_hit.set++;
			for(uint32_t index = 0; index < ptentries.size(); ++index)
			{
				// stats.register_prefetch_hit.set_total++;
				ptentries[index]->pf_cache_hit = true;
				// MYLOG("pref_hit PT hit. pref with act_idx %u act %d", ptentries[index]->action_index, Actions[ptentries[index]->action_index]);
			}
		}
	}
	std::vector<Scooby_PTEntry*> Pythia::search_pt(uint64_t address, bool search_all) {
		std::vector<Scooby_PTEntry*> entries;
		for(uint32_t index = 0; index < prefetch_tracker.size(); ++index)
		{
			if(prefetch_tracker[index]->address == address)
			{
				entries.push_back(prefetch_tracker[index]);
				if(!search_all) break;
			}
		}
		return entries;
	}
	void Pythia::update_stats(uint32_t state, uint32_t action_index, uint32_t pref_degree) {
		auto it = state_action_dist.find(state);
		if(it != state_action_dist.end())
		{
			it->second[action_index]++;
		}
		else
		{
			std::vector<uint64_t> act_dist;
			act_dist.resize(knob::scooby_max_actions, 0);
			act_dist[action_index]++;
			state_action_dist.insert(std::pair<uint32_t, std::vector<uint64_t> >(state, act_dist));
		}
	}
	void Pythia::update_stats(State *state, uint32_t action_index, uint32_t degree) {
		string state_str = state->to_string();
		auto it = state_action_dist2.find(state_str);
		if(it != state_action_dist2.end())
		{
			it->second[action_index]++;
			it->second[knob::scooby_max_actions]++; /* counts total occurences of this state */
		}
		else
		{
			std::vector<uint64_t> act_dist;
			act_dist.resize(knob::scooby_max_actions+1, 0);
			act_dist[action_index]++;
			act_dist[knob::scooby_max_actions]++; /* counts total occurences of this state */
			state_action_dist2.insert(std::pair<string, std::vector<uint64_t> >(state_str, act_dist));
		}

		auto it2 = action_deg_dist.find(getAction(action_index));
		if(it2 != action_deg_dist.end())
		{
			it2->second[degree]++;
		}
		else
		{
			std::vector<uint64_t> deg_dist;
			deg_dist.resize(MAX_SCOOBY_DEGREE, 0);
			deg_dist[degree]++;
			action_deg_dist.insert(std::pair<int32_t, std::vector<uint64_t> >(getAction(action_index), deg_dist));
		}
	}
	int32_t Pythia::getAction(uint32_t action_index) {
		assert(action_index < Actions.size());
		return Actions[action_index];
	}
	void Pythia::track_in_st(uint64_t page, uint32_t pred_offset, int32_t pref_offset) {
		auto st_index = find_if(signature_table.begin(), signature_table.end(), [page](Scooby_STEntry *stentry){return stentry->page == page;});
		if(st_index != signature_table.end())
		{
			(*st_index)->track_prefetch(pred_offset, pref_offset);
		}
	}

} // end namespace prefetch
} // end namespace gem5
