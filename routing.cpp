//#include "routing.h"
#include "routing_permutations.h"

vector<vector<int>> read_csv_matrix(const char *path) {
	vector<vector<int>> matrix;
	ifstream file(path);
	string line;
	string cell;

	//for every row
	int i = 0;
	while (file >> line) {
		matrix.push_back(vector<int>());
		vector<int> *row = &matrix[i++];
		
		stringstream linestream(line);
		int val;
		
		while (getline(linestream, cell, ',')) {
			if (isdigit(cell.at(0)))
				val = stoi(cell);
			else
				val = -1;
			row->push_back(val);
//#ifdef DEBUG
//			printf("%d,", val);
//#endif
		}
//#ifdef DEBUG
//		printf("\n");
//#endif
	}

	return matrix;
}

void update_rout(vector<rout*> &routs, node* n) {
	rout* trout = NULL;

	for (int i = 0; i < routs.size(); i++)
		if (routs[i]->id == n->rout_id) {
			trout = routs[i];
			break;
		}
	
	if (trout == NULL) {
		//trout = (rout*)malloc(sizeof(rout));
		trout = new rout;
		trout->id = n->rout_id;
		trout->end_points.push_back(n);
		trout->bb.imin = trout->bb.imax = n->i;
		trout->bb.jmin = trout->bb.jmax = n->j;
		trout->max_dist = -1;
		routs.push_back(trout);
	}
	else {
		trout->end_points.push_back(n);
		
		//update bounding box
		trout->bb.imin = __min(trout->bb.imin, n->i);
		trout->bb.imax = __max(trout->bb.imax, n->i);
		trout->bb.jmin = __min(trout->bb.jmin, n->j);
		trout->bb.jmax = __max(trout->bb.jmax, n->j);

		//update max distance
		int max_dist = trout->max_dist;
		for (int i = 0; i < trout->end_points.size() - 1; i++) {
			node* o = trout->end_points[i]; //the other end point
			max_dist = max(max_dist, manhattan_dist(n->i, n->j, o->i, o->j));
		}
		trout->max_dist = max_dist;
	}
}

routing_matrix* get_world_matrix(vector<vector<int>> data) {
	int data_rows = data.size();

	//routing_matrix* matrix = (routing_matrix*)malloc(sizeof(routing_matrix));
	routing_matrix* matrix = new routing_matrix;
	if (matrix == NULL)
		return NULL;

	matrix->rows = data_rows;
	//matrix->grid = (node**)malloc(sizeof(node*) * matrix->rows);
	matrix->grid = new node*[matrix->rows];
	if (matrix->grid == NULL) {
		delete matrix;
		return NULL;
	}
	
	matrix->cols = data[0].size();
	for (int i = 0; i < matrix->rows; i++) {
		//matrix->grid[i] = (node*)malloc(sizeof(node) * matrix->cols);
		matrix->grid[i] = new node[matrix->cols];
		for (int j = 0; j < matrix->cols; j++) {
			node* n = &(matrix->grid[i][j]);
			n->i = i;
			n->j = j;
			n->parent = NULL;
			n->rout_id = data[i][j];
			n->state = STATE::WHITE;
			n->dir = -1;
			if (data[i][j] == 0) {
				n->end_point = 0;
			}
			else if (data[i][j] > 0) {
				n->end_point = 1;
				update_rout(matrix->routs, n);
			}
		}
	}

	//update number of other routs end points in the bb of each rout
	//and initialize routed flags
	for (int k = 0; k < matrix->routs.size(); k++) {
		rout* r = matrix->routs[k];

		for (int i = 0; i < r->end_points.size(); i++)
			r->routed.push_back(false);
		r->unrouted_count = r->end_points.size();

		int cnt = 0;
		for (int i = r->bb.imin; i <= r->bb.imax; i++) {
			for (int j = r->bb.jmin; j <= r->bb.jmax; j++) {
				int rout_id = matrix->grid[i][j].rout_id;
				if (rout_id > 0 && rout_id != r->id)
					cnt++;
			}
		}
		r->bb.other_routs_count = cnt;
	}

	for (int i = 0; i < matrix->rows; i++) {
		for (int j = 0; j < matrix->cols; j++) {
			node* n = &(matrix->grid[i][j]);
			int bbCnt = 0;
			for (int k = 0; k < matrix->routs.size(); k++) {
				rout* r = matrix->routs[k];
				if (i >= r->bb.imin && i <= r->bb.imax && j >= r->bb.jmin && j <= r->bb.jmax)
					bbCnt++;
			}
			n->bbCount = bbCnt;
		}
	}

	//sort routs by id
	sort(matrix->routs.begin(), matrix->routs.end(), routs_id_comp);

	return matrix;
}

inline int is_inside(routing_matrix* matrix, int i, int j) {
	return (0 <= i && i < matrix->rows && 0 <= j && j < matrix->cols);
}

inline int check_neighbor(routing_matrix* matrix, rout* r, node* n) {
	if (n->state != WHITE)
		return 0;

	if (n->rout_id != 0) {
		if (n->end_point && n->rout_id == r->id)
			return 1;
		return 0;
	}
	
	int dirRow[] = { -1, -1, -1, 0, 0, 1, 1, 1 };
	int dirCol[] = { -1, 0, 1, -1, 1, -1, 0, 1 };

	//test proximity
	for (int k = 0; k < 8; k++) {
		int newi = n->i + dirRow[k];
		int newj = n->j + dirCol[k];
		if (is_inside(matrix, newi, newj) && matrix->grid[newi][newj].rout_id != 0 && matrix->grid[newi][newj].rout_id != r->id)
			return 0;
	}

	return 1;
}

inline void update_dist_to_endpoints(node* n, rout* r) {
	n->dist_to_endpoints.clear();
	for (int i = 0; i < r->end_points.size(); i++) {
		if (r->routed[i] == false) {
			node* endp = r->end_points[i];
			n->dist_to_endpoints.push_back(pair<int, node*>(manhattan_dist(endp->i, endp->j, n->i, n->j), endp));
		}
	}
	sort(n->dist_to_endpoints.begin(), n->dist_to_endpoints.end());
}

void expand(routing_matrix *matrix, priority_queue<node*, vector<node*>, routing_heuristic>& frontier, rout* r, node* n) {

	int dirRow[] = { 0, 1, 0, -1 };
	int dirCol[] = { 1, 0, -1, 0 };

	for (int k = 0; k < 4; k++) {
		int newi = n->i + dirRow[k];
		int newj = n->j + dirCol[k];
		if (is_inside(matrix, newi, newj)) {
			node* neighbor = &(matrix->grid[newi][newj]);
			
			if (check_neighbor(matrix, r, neighbor)) {
				//when expand a node it's first time and last time for it for this active rout
				//so first update distances vector to active rout
				update_dist_to_endpoints(neighbor, r);
				neighbor->parent = n;
				neighbor->state = GRAY;
				neighbor->dir = k;
				frontier.push(neighbor);
			}
		}
	}
}

/*

*/
void update_frontier(priority_queue<node*, vector<node*>, routing_heuristic> &frontier, rout *r, node* end_point) {
	priority_queue<node*, vector<node*>, routing_heuristic> tmp;
	
	while (!frontier.empty()) {
		node* n = frontier.top();
		frontier.pop();

		//remove last routed end point from distances
		//for (vector<pair<int, node*>>::reverse_iterator ite = n->dist_to_endpoints.rbegin(); ite < n->dist_to_endpoints.rend(); ite++) {
		for (int i = 0; i < n->dist_to_endpoints.size(); i++) {
			if (n->dist_to_endpoints[i].second == end_point) {
				n->dist_to_endpoints.erase(n->dist_to_endpoints.begin() + i);
				break;
			}
		}

		tmp.push(n);
	}

	frontier.swap(tmp);
}

int astar(routing_matrix *matrix, rout* r, node **closest) {
	priority_queue<node*, vector<node*>, routing_heuristic> frontier;

	node* start = r->end_points[0];
	r->routed[0] = true;
	r->unrouted_count--;
	update_dist_to_endpoints(start, r);

	frontier.push(start);

	node* pred = NULL;
	node* actual = NULL;
	bool finish = false;
	int dist_min = INT_MAX;

	while (!frontier.empty()) {
		actual = frontier.top();
		frontier.pop();

		actual->rout_id = r->id;
		actual->state = BLACK;
		//actual->parent = pred;

		if (actual->dist_to_endpoints[0].first < dist_min) {
			*closest = actual;
			dist_min = actual->dist_to_endpoints[0].first;
		}

		//print_grid(matrix);

		if (actual->end_point && pred != NULL) {
			r->unrouted_count--;
			if (r->unrouted_count == 0) {
				finish = true;
				break;
			}

			//set this end point to routed
			for (int i = 0; i < r->end_points.size(); i++)
				if (r->end_points[i] == actual)
					r->routed[i] = true;

			update_frontier(frontier, r, actual);
		}

		//expand frontier
		expand(matrix, frontier, r, actual);

		pred = actual;
	}

	if (!finish)
		return 1;

	//construct routing path
	for (int i = 0; i < r->end_points.size(); i++) {
		pred = r->end_points[i];
		while (pred) {
			if (pred->state == ROUTED)
				break;
			pred->state = ROUTED;
			pred = pred->parent;
		}
	}

	return 0;
}

void clear_grid(routing_matrix* matrix, rout* r) {
	for (int i = 0; i < matrix->rows; i++) {
		for (int j = 0; j < matrix->cols; j++) {
			node* n = &(matrix->grid[i][j]);
			//if (i == 25 && j == 30)
				//i = 25;
			if (n->state != ROUTED && !n->end_point)
				if (n->state == GRAY || n->rout_id == r->id) {
					n->rout_id = 0;
					n->state = WHITE;
					n->parent = NULL;
					n->dir = -1;
				}
		}
	}
}

void clear_routs(routing_matrix* matrix, list<rout*> routs) {
	for (int i = 0; i < matrix->rows; i++) {
		for (int j = 0; j < matrix->cols; j++) {
			node* n = &(matrix->grid[i][j]);
			for (list<rout*>::iterator it = routs.begin(); it != routs.end(); it++) {
				if (n->rout_id == (*it)->id || n->state == GRAY) {
					n->state = WHITE;
					n->parent = NULL;
					n->dir = -1;
					if (!n->end_point)
						n->rout_id = 0;
					break;
				}
			}
		}
	}

	for (list<rout*>::iterator it = routs.begin(); it != routs.end(); it++) {
		rout* r = *it;
		r->unrouted_count = r->end_points.size();
		for (int i = 0; i < r->end_points.size(); i++) {
			r->routed[i] = false;
		}
	}
}

list<rout*> find_blocking_routs(routing_matrix *matrix, rout *r, const vector<bool>& rerouted, node *closest) {
	//first of all check all end points that have not been routed
	//int dirRow[] = { -2, -2, -2, -2, -2, -1, -1,  0, 0,  1, 1,  2,  2, 2, 2, 2 };
	//int dirCol[] = { -2, -1,  0,  1,  2, -2,  2, -2, 2, -2, 2, -2, -1, 0, 1, 2 };
	int dirRow[] = { -2,  0, 0, 2 };
	int dirCol[] = {  0, -2, 2, 0 };

	set<rout*, routing_comparer> blocking_routs;
	set<rout*, routing_comparer> rerouted_blocking_routs;

	//first check if rout is blocked at end points
	for (int i = 0; i < r->end_points.size(); i++) {
		node* end_point = r->end_points[i];
		if (end_point->parent == NULL) {//take start point in consideration
			bool is_blocked = true;
			for (int k = 0; k < 4; k++) {
				int newi = end_point->i + dirRow[k];
				int newj = end_point->j + dirCol[k];
				if (is_inside(matrix, newi, newj)) {
					node* n = &(matrix->grid[newi][newj]);
					if (check_neighbor(matrix, r, n) || n->rout_id == r->id) {
						is_blocked = false;
						break;
					}
					else if (!n->end_point && n->rout_id > 0) {
						if (rerouted[n->rout_id - 1])
							rerouted_blocking_routs.insert(matrix->routs[n->rout_id - 1]);
						else
							blocking_routs.insert(matrix->routs[n->rout_id - 1]);
					}
				}
			}
			if (!is_blocked) {
				blocking_routs.clear();
				rerouted_blocking_routs.clear();
			}
			else
				break;
		} 
	}

	list<rout*> blocking_routs_list;
	if (blocking_routs.size() == 0 && rerouted_blocking_routs.size() == 0) {
		
		//take the closest BLACK node and rerout the rout which is the closest to goal
		int min_dist = INT_MAX;
		rout* closest_rout = NULL;
		for (int k = 0; k < 4; k++) {
			int newi = closest->i + dirRow[k];
			int newj = closest->j + dirCol[k];
			if (is_inside(matrix, newi, newj)) {
				node* n = &(matrix->grid[newi][newj]);
				if (!n->end_point && n->rout_id > 0 && n->rout_id != r->id) {
					update_dist_to_endpoints(n, r);
					if (n->dist_to_endpoints[0].first < min_dist) {
						min_dist = n->dist_to_endpoints[0].first;
						closest_rout = matrix->routs[n->rout_id - 1];
					}
				}
			}
		}
		if (closest_rout != NULL)
			blocking_routs_list.push_back(closest_rout);
	}
	else if (blocking_routs.size() == 0) {//then is blocked by rerouted routs so they need to be rerouted again
		for (set<rout*, routing_comparer>::reverse_iterator it = rerouted_blocking_routs.rbegin(); it != rerouted_blocking_routs.rend(); it++)
			blocking_routs_list.push_back(*it);
	}
	else {
		for (set<rout*, routing_comparer>::reverse_iterator it = blocking_routs.rbegin(); it != blocking_routs.rend(); it++)
			blocking_routs_list.push_back(*it);
	}

	return blocking_routs_list;
}



void routing(routing_matrix *matrix) {
	routing_permutation routing_perm(matrix);
	int test = 0;

	while (!routing_perm.finish()) {
		rout* r = routing_perm.next_to_rout();

		node* closest = NULL;
		int status = astar(matrix, r, &closest);

		if (status == 0) {
			clear_grid(matrix, r);
			//routed.push_back(r);
			routing_perm.push_routed(r);
		}
		else { //routing failed
			//printf("routed failed at rout with id=%d\n", r->id);
			// when a rout failed, first search the other routs which are blocking this rout
			// reset that routs, start routing first this rout and rerout them after that
			
			// first set this routing perm failure
			routing_perm.mark_failure(r);

			// search for blocking routs
			list<rout*> to_rerout = find_blocking_routs(matrix, r, routing_perm.rerouted, closest);
			list<rout*> to_clear = routing_perm.set_to_rerout(to_rerout, r);

			// clear routs
			clear_routs(matrix, to_clear);
		} 
#ifdef DEBUG
		//print_grid(matrix);
#endif
	}
}

set<int> BFS(routing_matrix *matrix, node* start) {
	int dirRow[] = { -1, -1, -1,  0, 0,  1, 1, 1 };
	int dirCol[] = { -1,  0,  1, -1, 1, -1, 0, 1 };
	queue<node*> q;
	set<int> routs_set;

	q.push(start);
	routs_set.insert(start->rout_id);

	while (!q.empty()) {
		node* n = q.front();
		q.pop();

		for (int k = 0; k < 8; k++) {
			int newi = n->i + dirRow[k];
			int newj = n->j + dirCol[k];
			if (is_inside(matrix, newi, newj)) {
				node* neigh = &(matrix->grid[newi][newj]);
				if (neigh->rout_id > 0 && neigh->state == WHITE) {
					q.push(neigh);
					neigh->state = GRAY;
					if (neigh->rout_id != n->rout_id)
						routs_set.insert(neigh->rout_id);
				}
			}
		}
	}

	return routs_set;
}

int check_routing(routing_matrix *matrix) {
	vector<set<int>> sets;
	
	for (int i = 0; i < matrix->rows; i++) {
		for (int j = 0; j < matrix->cols; j++) {
			node* n = &(matrix->grid[i][j]);
			if (n->rout_id > 0 && n->state == WHITE) {
				sets.push_back(BFS(matrix, n));
			}
		}
	}

	printf("Intersecting routes:\n");
	int k = 0;
	for (int i = 0; i < sets.size(); i++) {
		if (sets[i].size() > 1) {
			printf("Route %d: pins ", ++k);
			for (set<int>::iterator iter = sets[i].begin(); iter != sets[i].end(); iter++) {
				printf("%d, ", *iter);
			}
			printf("\n");
		}
	}

	return k == 0;//good routing if k == 0
}

int main() {

	printf("Step 1:\n");
	vector<vector<int>> data = read_csv_matrix("Step_One.csv");
	routing_matrix* matrix = get_world_matrix(data);
	
	//test_read(matrix);

	//printf("Input matrix:\n");
	//print_grid(matrix);
	routing(matrix);
	printf("Routed matrix:\n");
	print_grid(matrix);

	printf("Step 3:\n");
	vector<vector<int>> data3 = read_csv_matrix("Step_Three.csv");
	routing_matrix* matrix3 = get_world_matrix(data3);

	printf("Input matrix:\n");
	print_grid(matrix3);
	check_routing(matrix3);

	printf("Step 2:\n");
	vector<vector<int>> data2 = read_csv_matrix("Step_Two.csv");
	routing_matrix* matrix2 = get_world_matrix(data2);
	printf("Routed matrix:\n");
	routing(matrix2);
	print_grid(matrix2);

 	return 0;
}

