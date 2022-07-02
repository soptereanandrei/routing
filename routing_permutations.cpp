#include "routing_permutations.h"


routing_permutation::routing_permutation(routing_matrix* matrix) {
	this->matrix = matrix;
	
	for (int i = 0; i < matrix->routs.size(); i++) {
		this->to_rout.push_back(matrix->routs[i]);
	}
	this->to_rout.sort(routs_comp);
	this->to_rout.reverse();

	this->rerouting = false;
	for (int i = 0; i < matrix->routs.size(); i++) {
		this->rerouted.push_back(false);
	}
}

bool routing_permutation::finish() {
	return this->to_rout.empty() && this->to_rerout.empty();
}

rout* routing_permutation::next_to_rout() {
	rout* r;
	
	if (!this->to_rerout.empty()) {
		r = this->to_rerout.front();
		this->to_rerout.pop_front();
	}
	else {
		r = this->to_rout.front();
		this->to_rout.pop_front();

		//if (this->rerouted[r->id - 1] == false && this->rerouting) {
		if (this->rerouting) {
			this->rerouting = false;
			this->set_finish_rerouting();
		}
	}

	return r;
}

void routing_permutation::push_routed(rout* r) {
	routed.push_back(r);
}

list<rout*> routing_permutation::set_to_rerout(list<rout*> to_rerout, rout *head) {
	//flag that we start rerouting process
	this->rerouting = true;
	
	//clear from routed all routs that have to be rerouted
	list<rout*> to_clear;
	for (list<rout*>::reverse_iterator it = to_rerout.rbegin(); it != to_rerout.rend(); it++) {
		this->routed.remove(*it);
		this->to_rerout.push_front(*it);//!!!!!!!!!!!!!!
		this->rerouted[(*it)->id - 1] = true;
		to_clear.push_front(*it);
	}

	this->to_rerout.push_front(head);
	to_clear.push_front(head);

	this->next_perm(to_clear);
	
	return to_clear;
}

int perm(perm_param& params, string chain, int pos) {
	if (pos >= params.perm_len)
		return 1;

	for (int i = 0; i < params.perm_len; i++) {
		if (!params.taken[i]) {
			params.taken[i] = true;
			params.perm_vec[pos] = i;

			//check permutation
			string newchain(chain);
			newchain.append(params.ids_table[i]);
			if (params.perm->explored.count(newchain) == 0) {
				if (perm(params, newchain, pos + 1))
					return 1;
			}

			if (pos < params.rerout_index) //need to rerout this position
				params.rerout_index = pos;

			params.taken[i] = false;
		}
	}

	return 0;
}

void routing_permutation::next_perm(list<rout*>& to_clear) {
	// find next unexplored permutation
	// take in consideration only the routs that was routed before and rout that want to be rerouted
	// it doesn't make sense to perm all routs, rout that are in to_rout list, they was never routed
	// so them not need to be perm for now
	perm_param params;

	params.perm = this;

	params.perm_len = this->routed.size() + this->to_rerout.size();

	params.perm_vec = new int[params.perm_len];
	params.taken = new bool[params.perm_len]{ false };//used vector
	//params.ids_table = new string[params.perm_len];

	int i = 0;
	for (list<rout*>::iterator iter = this->routed.begin(); iter != this->routed.end(); iter++) {
		//params.ids_table[i].assign(to_string((*iter)->id));
		params.ids_table.push_back(to_string((*iter)->id));
		i++;
	}
	for (list<rout*>::iterator iter = this->to_rerout.begin(); iter != this->to_rerout.end(); iter++) {
		//params.ids_table[i].assign(to_string((*iter)->id));
		params.ids_table.push_back(to_string((*iter)->id));
		i++;
	}

	params.rerout_index = this->routed.size();

	string chain;
	if (!perm(params, chain, 0)) {
		printf("Impossible to rout\n");
		to_rout.clear();//to finish
		to_rerout.clear();
		return;
	}

	// first take routs that was routed and need to rerout
	//list<rout*> to_clear;
	i = this->routed.size() - 1;
	while (i >= params.rerout_index) {
		to_clear.push_front(this->routed.back());
		this->routed.pop_back();
		i--;
	}

	// reconstruct lists based on the perm
	this->to_rerout.clear();
	for (i = params.rerout_index; i < params.perm_len; i++) {
		int id = stoi(params.ids_table[params.perm_vec[i]]);
		this->to_rerout.push_back(this->matrix->routs[id - 1]);
	}

	delete[] params.perm_vec;
	delete[] params.taken;
	//delete[] params.ids_table;

	//return to_clear;
}

void routing_permutation::mark_failure(rout* r) {
	//construct chain of this routing permutation

	string chain;
	for (list<rout*>::iterator iter = this->routed.begin(); iter != this->routed.end(); iter++) {
		chain.append(to_string((*iter)->id));
	}
	chain.append(to_string(r->id));

	this->explored.insert(chain);
}

void routing_permutation::set_finish_rerouting() {
	for (int i = 0; i < this->rerouted.size(); i++) {
		this->rerouted[i] = false;
	}
}