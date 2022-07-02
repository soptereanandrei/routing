#pragma once

#include "routing.h"
#include <unordered_set>


struct routing_permutation {
	routing_matrix* matrix;//reference to the routing matrix
	list<rout*> routed;//list of routs that are already routed
	list<rout*> to_rout;//list of routs that need to be routed
	list<rout*> to_rerout;//list of routs that need to be rerouted
	vector<bool> rerouted;//flag if a rout was recently rerouted, to try to avoid rerouting them
	bool rerouting;//flag if we are in rerouting process
	unordered_set<string> explored;//hash map with explored permutations

	routing_permutation(routing_matrix* matrix);
	
	bool finish();
	rout* next_to_rout();
	void push_routed(rout *r);
	list<rout*> set_to_rerout(list<rout*> to_rerout, rout* head);
	void next_perm(list<rout*>& to_clear);
	void mark_failure(rout *r);
	void set_finish_rerouting();
};

struct perm_param {
	routing_permutation* perm;
	int perm_len;
	int* perm_vec;
	bool* taken;
	vector<string> ids_table;
	int rerout_index;
};