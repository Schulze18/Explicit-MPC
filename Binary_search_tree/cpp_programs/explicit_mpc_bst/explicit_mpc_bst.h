//Header file with struct specification

#ifndef EXPLICIT_MPC_BST_H
#define EXPLICIT_MPC_BST_H

//Necessary Includes
#include <iostream>
#include <vector>
#include <fstream>

struct struct_bst {
	std::vector<double> A;
	double b;
	int left;
	int right;
	int parent_node;
	std::vector<int> regions;
};

struct struct_regions {
	std::vector< std::vector<double> > set_A;
	std::vector<double> set_b;
	std::vector<double> Kx;
	std::vector<double> Kc;
};

void read_double_from_file(double *element, std::ifstream *file);
void read_int_from_file(int *element, std::ifstream *file);
void get_bst_from_file(std::vector<struct_bst> *bst_nodes , const char *filename);
void get_regions_from_file(std::vector<struct_regions> *regions, const char *filename);
int index_region_evaluate_bst(double state[], std::vector<struct_bst> *bst_nodes);
double *calculate_control_explicit_bst(double state[], std::vector<struct_regions> *regions, int index_region);

#endif
