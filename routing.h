#pragma once

#include <stdio.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <list>
#include <queue>
#include <set>


using namespace std;

#define DEBUG

enum STATE {WHITE, GRAY, BLACK, ROUTED};

struct node {
	int i;
	int j;
	int rout_id;
	int end_point;
	STATE state;
	vector<pair<int, node*>> dist_to_endpoints;//this is update for every rout when it will be routed
	node* parent;
	int dir;//direction to come from parent
	int bbCount;//number of bounding boxes in which this node is
};

struct bounding_box {
	int imin;
	int jmin;
	int imax;
	int jmax;
	int other_routs_count; // number of other routs endpoint presented in the bounding box
};

struct rout {
	int id;
	vector<node*> end_points;
	vector<bool> routed; //flag, which end points are already routed
	int unrouted_count; //count number of end points that are not already routed
	bounding_box bb;
	int max_dist; //maximum distance between end points
};

struct routing_matrix {
	node** grid;
	int rows;
	int cols;
	vector<rout*> routs;
};


inline int manhattan_dist(int i1, int j1, int i2, int j2) {
	return abs(i1 - i2) + abs(j1 - j2);
}

inline bool routs_id_comp(rout* a, rout* b) {
	return (a->id < b->id);
}

/*
It is used to order routing
Criteria:
1. Number of end points (less points routed before)
2. Number of other routs's end points in Boundary Box (less points routed before)
3. Max distance between 2 end points (less distance routed before)
*/

inline bool routs_comp(rout* a, rout* b) {
	int asize = a->end_points.size();
	int bsize = b->end_points.size();

	//first criteria
	if (asize > bsize)
		return true;
	else if (asize == bsize) {
		//second criteria
		int a_endpoints = a->bb.other_routs_count;
		int b_endpoints = b->bb.other_routs_count;

		if (a_endpoints > b_endpoints)
			return true;
		else if (a_endpoints == b_endpoints) {
			//third criteria
			int a_dist = a->max_dist;
			int b_dist = b->max_dist;

			if (a_dist > b_dist)
				return true;
			else if (a_dist == b_dist)
				return a->id > b->id;//ascending id ordering
			else
				return false;
		}
		else
			return false;
	}
	else
		return false;
}

struct routing_comparer {
	bool operator()(rout* a, rout* b) const {
		return routs_comp(a, b);
	}
};

/*
The heuristic is the minimal manhattan distance to one of the end points
*/
struct routing_heuristic {
	bool operator()(node* a, node* b) {

		int adist = (a->dist_to_endpoints[0]).first;
		int bdist = (b->dist_to_endpoints[0]).first;
		if (adist > bdist)
			return true;
		else if (adist == bdist) {
			if (a->bbCount > b->bbCount)
				return true;
			else if (a->bbCount == b->bbCount) {
				// try to keep straight lines to save space
				if (a->parent && b->parent) {
					int dir_parent_a = a->parent->dir;
					int dir_parent_b = b->parent->dir;

					if ((dir_parent_a == a->dir && dir_parent_b == b->dir) || (dir_parent_a != a->dir && dir_parent_b != b->dir))
						return (a->dist_to_endpoints[0]).second < (b->dist_to_endpoints[0]).second;
					else
						return dir_parent_a != a->dir;
				}
				else if (a->parent)
					return false;
				else if (b->parent)
					return true;
				else
					return (a->dist_to_endpoints[0]).second < (b->dist_to_endpoints[0]).second;
			}
			else
				return false;
		}
		else
			return false;
	}
};



#ifdef DEBUG

inline void test_read(routing_matrix* matrix) {
	printf("Number of routs = %ld\n", matrix->routs.size());

	for (int i = 0; i < matrix->routs.size(); i++) {
		rout* r = matrix->routs[i];

		printf("---------------------\n");
		printf("rout with id = %d\n", r->id);
		printf("Have end points in:\n");
		for (int j = 0; j < r->end_points.size(); j++) {
			node* e = r->end_points[j];
			printf("(%d, %d), with rood_id = %d, is marked to be end_point = %d\n", e->i, e->j, e->rout_id, e->end_point);
		}
		printf("Bounding box:");
		printf("imin = %d jmin = %d\nimax = %d jmax = %d\n", r->bb.imin, r->bb.jmin, r->bb.imax, r->bb.jmax);
		printf("Max dist = %d\n", r->max_dist);
	}
}

inline void print_grid(routing_matrix* matrix) {
	//printf("----------------\n");
	printf("     ");
	for (int j = 0; j < matrix->cols; j++)
		printf("%2d ", j);
	printf("\n");
	printf("    ");
	for (int j = 0; j < matrix->cols; j++)
		printf("___");
	printf("\n");
	for (int i = 0; i < matrix->rows; i++) {
		printf("%2d | ", i);
		for (int j = 0; j < matrix->cols; j++) {
			printf("%2d ", matrix->grid[i][j].rout_id);
		}
		printf("\n");
	}
}

#endif

//struct routing_heuristic {
//	bool operator()(node* a, node* b) {
//
//		int adist = (a->dist_to_endpoints[0]).first;
//		int bdist = (b->dist_to_endpoints[0]).first;
//		if (adist > bdist)
//			return true;
//		else if (adist == bdist) {
//
//
//			// try to keep straight lines to save space
//			if (a->parent && b->parent) {
//				int dir_parent_a = a->parent->dir;
//				int dir_parent_b = b->parent->dir;
//
//				if ((dir_parent_a == a->dir && dir_parent_b == b->dir) || (dir_parent_a != a->dir && dir_parent_b != b->dir))
//					return (a->dist_to_endpoints[0]).second < (b->dist_to_endpoints[0]).second;
//				else
//					return dir_parent_a != a->dir;
//			}
//			else if (a->parent)
//				return false;
//			else if (b->parent)
//				return true;
//			else
//				return (a->dist_to_endpoints[0]).second < (b->dist_to_endpoints[0]).second;
//		}
//		else
//			return false;
//	}
//};