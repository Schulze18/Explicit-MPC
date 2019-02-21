//Header file with struct specification

#ifndef EXPLICIT_MPC_BST_H
#define EXPLICIT_MPC_BST_H

//Necessary Includes
#include <iostream>
#include <vector>

struct struct_control_param{

	int number_states;
	int number_controls_actions;

	int total_regions;
	int total_nodes;
	int total_ineq;

	int number_fields_bst;
	int number_fields_regions;
	int number_fields_ineq;
};

struct struct_regions {
	std::vector<double> Kx;
	std::vector<double> Kc;
};

struct struct_ineq_set {
	std::vector<double> A;
	double b;
};

struct struct_bst {
	int ineq_index;
	int left;
	int right;
	int parent_node;
	std::vector<int> regions;
};

void read_double_from_file(double *element, std::ifstream *file);

void read_int_from_file(int *element, std::ifstream *file);

void get_regions_from_file(std::vector<struct_regions> *regions, const char *filename);

void get_bst_from_file(std::vector<struct_bst> *bst_nodes , const char *filename);

void get_ineq_set_from_file(std::vector<struct_ineq_set> *ineq_set, const char *filename);

void get_control_param_file(struct_control_param *control_param, const char *filename);

int index_region_evaluate_bst(double state[], size_t number_states, std::vector<struct_bst> *bst_nodes, std::vector<struct_ineq_set> *ineq_set); 

void calculate_control_explicit_bst(double *array_control_action, double state[], size_t number_states, size_t number_controls_actions, std::vector<struct_regions> *regions, int index_region);

#endif
