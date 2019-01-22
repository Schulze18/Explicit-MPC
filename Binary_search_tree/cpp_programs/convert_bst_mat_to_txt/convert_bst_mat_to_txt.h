//Header file with struct specification

#ifndef CONVERT_BST_MAT_TO_TXT_H
#define CONVERT_BST_MAT_TO_TXT_H

//Necessary Include
#include "explicit_mpc_bst.h"
#include "mat.h"
#include <iostream>
#include <vector>
#include <fstream>

void write_double_to_file(double *element, std::ifstream *file);
void write_int_to_file(int *element, std::ifstream *file);
void get_struct_bst_from_mat(std::vector<struct_bst> *bst_nodes , const char *filename);
void get_struct_regions_from_mat(std::vector<struct_regions> *regions, const char *filename);
void export_bst_to_file(std::vector<struct_bst> *bst_nodes , const char *filename);
void export_regions_to_file(std::vector<struct_regions> *regions, const char *filename);


#endif
