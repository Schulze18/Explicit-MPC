#include "C:\Users\schul\Documents\UDESC\TCC\TCC1\Programas MATLAB\GitHub\Explicit-MPC\Binary_search_tree\cpp_programs\convert_bst_mat_to_txt\convert_bst_mat_to_txt.h"
#include "C:\Users\schul\Documents\UDESC\TCC\TCC1\Programas MATLAB\GitHub\Explicit-MPC\Binary_search_tree\cpp_programs\explicit_mpc_bst\explicit_mpc_bst.h"
#include "C:\Users\schul\Documents\UDESC\TCC\TCC1\Programas MATLAB\GitHub\Explicit-MPC\Binary_search_tree\cpp_programs\convert_bst_mat_to_txt\mat.h"
#include <iostream>
#include <vector>
#include <fstream>

//int number_fields_bst;
//int number_fields_regions;
//int max_number_ineq;
/*int total_nodes;
int total_regions;
int number_states;
int number_controls_actions;
int number_fields_bst;
int number_fields_regions;
int max_number_ineq;
*/

void write_double_to_file(double *element, std::ofstream *file) {
	(*file).write((char*)&(*element), sizeof(*element));
}

void write_int_to_file(int *element, std::ofstream *file) {
	(*file).write((char*)&(*element), sizeof(*element));
}
/*
void export_bst_to_file(std::vector<struct_bst> *bst_nodes , const char *filename){

    std::ofstream file_bst(filename, std::ios::out | std::ios::binary);

    write_int_to_file(&total_nodes,&file_bst);
    write_int_to_file(&number_states,&file_bst);

	for (int index_node = 0; index_node < total_nodes; index_node++){

        //Export A
        for (int it_A = 0; it_A < number_states; it_A++){
            double element_A;
			element_A = (*bst_nodes)[index_node].A[it_A];
			write_double_to_file(&element_A,&file_bst);
        }

        //Export b
        write_double_to_file(&((*bst_nodes)[index_node].b),&file_bst);

        //Export left
        write_int_to_file(&((*bst_nodes)[index_node].left),&file_bst);

        //Export right
        write_int_to_file(&((*bst_nodes)[index_node].right),&file_bst);

        //Export Number of regions
        int number_regions_node = (*bst_nodes)[index_node].regions.size();
        write_int_to_file(&number_regions_node,&file_bst);

        //Export regions
        for (int it_region = 0; it_region < number_regions_node; it_region++){
            int element_region = (*bst_nodes)[index_node].regions[it_region];
                write_int_to_file(&element_region,&file_bst);
        }
	}
    file_bst.close();

}

void export_regions_to_file(std::vector<struct_regions> *regions, const char *filename){

    std::ofstream file_regions(filename, std::ios::out | std::ios::binary);

    //Export total number of regions
    write_int_to_file(&total_regions,&file_regions);

    //Export number of controls actions u
    write_int_to_file(&number_controls_actions, &file_regions);

    for (int index_region = 0; index_region < total_regions; index_region++) {

        //Export number of inequation that define the region
        int number_inequation_region = (*regions)[index_region].set_A.size();
        write_int_to_file(&number_inequation_region,&file_regions);

        //Export inequations set A
        for (int it_ineq = 0; it_ineq < number_inequation_region; it_ineq++) {
			for (int it_element = 0; it_element < number_states; it_element++) {
                double element_ineq_A = (*regions)[index_region].set_A[it_ineq][it_element];
                write_double_to_file(&element_ineq_A,&file_regions);
            }
        }

        //Export b
        for (int it_ineq = 0; it_ineq < number_inequation_region; it_ineq++) {
            double element_b = (*regions)[index_region].set_b[it_ineq];
            write_double_to_file(&element_b,&file_regions);
        }

        //Export Kx
		for (int it_kx = 0; it_kx < number_controls_actions*number_states; it_kx++) {
            double element_kx = (*regions)[index_region].Kx[it_kx];
            write_double_to_file(&element_kx,&file_regions);
		}

        //Export Kc
		for (int it_kc = 0; it_kc < number_controls_actions; it_kc++) {
			double element_kc = (*regions)[index_region].Kc[it_kc];
            write_double_to_file(&element_kc,&file_regions);
		}
    }
    file_regions.close();
}*/

void get_struct_bst_from_mat(struct_control_param *control_param, std::vector<struct_bst> *bst_nodes, const char *filename){
    
	MATFile *pmat_bst;

    pmat_bst = matOpen(filename, "r");
	//Get struct from the .mat, it is maybe necessary to change the struct name
	mxArray *mat_data_nodes = matGetVariable(pmat_bst, "nodes");

    //Get all the data from the struct
	mxArray **bst_elements;
	bst_elements = (mxArray **)mxGetData(mat_data_nodes);

	(*control_param).number_fields_bst = mxGetN(mat_data_nodes);//mxGetNumberOfFields(mat_struct_regions);
	(*control_param).total_nodes = mxGetM(mat_data_nodes);//mxGetNumberOfElements(mat_struct_regions);


    for (int index_node = 0; index_node < (*control_param).total_nodes; index_node++) {
        (*bst_nodes).push_back(struct_bst());

        //Reading index ineq from .mat
		double *pointer_ineq_index = mxGetPr(bst_elements[2 * (*control_param).total_nodes + index_node]);
		int number_index = mxGetM(bst_elements[2 * (*control_param).total_nodes + index_node]);
		std::vector<int> temp_index(pointer_ineq_index, pointer_ineq_index + number_index);

		(*bst_nodes)[index_node].ineq_index = temp_index[number_index - 1];

        //Reading left node from .mat	
		(*bst_nodes)[index_node].left = *(mxGetPr(bst_elements[4 * (*control_param).total_nodes + index_node]));
		
		//Reading right node from .mat
		(*bst_nodes)[index_node].right = *(mxGetPr(bst_elements[5 * (*control_param).total_nodes + index_node]));

		//Reading parent node from .mat
		(*bst_nodes)[index_node].parent_node = *(mxGetPr(bst_elements[6 * (*control_param).total_nodes + index_node]));

		
		//Reading all the regions nodes
		double *pointer_regions = mxGetPr(bst_elements[3 * (*control_param).total_nodes + index_node]);
		int number_region_node = mxGetM(bst_elements[3 * (*control_param).total_nodes + index_node]);
		(*bst_nodes)[index_node].regions.assign(pointer_regions, pointer_regions + number_region_node);

    }
    matClose(pmat_bst);
}

void get_struct_regions_from_mat(struct_control_param *control_param, std::vector< struct_regions > *regions, const char (*filename)){

	//(*control_param).number_fields_bst = 1;

	MATFile *pmat_regions;
	
    pmat_regions = matOpen(filename, "r");
    //Get struct from the .mat, it is maybe necessary to change the struct name
    mxArray *mat_data_regions = matGetVariable(pmat_regions, "Regions");

	(*control_param).number_fields_regions = mxGetN(mat_data_regions);//mxGetNumberOfFields(mat_struct_regions);
	(*control_param).total_regions = mxGetM(mat_data_regions);//mxGetNumberOfElements(mat_struct_regions);
	
	mxArray **element_region;
	element_region = (mxArray **)mxGetData(mat_data_regions);

	(*control_param).number_states = mxGetN(element_region[2 * (*control_param).total_regions]);
	(*control_param).number_controls_actions = mxGetM(element_region[2 * (*control_param).total_regions]);
	
    for (int index_region = 0; index_region < (*control_param).total_regions; index_region++) {
        (*regions).push_back(struct_regions());
		
		//Reading Kx from .mat
		double *pointer_Kx = mxGetPr(element_region[2 * (*control_param).total_regions + index_region]);

		std::vector<double> temp_Kx;
		temp_Kx.assign(pointer_Kx, pointer_Kx + (*control_param).number_controls_actions * (*control_param).number_states);

		//Logic to sort the elements in the desired order
		for (int it_row_kx = 0; it_row_kx < (*control_param).number_controls_actions; it_row_kx++) {
			for (int it_col_kx = 0; it_col_kx < (*control_param).number_states; it_col_kx++) {
				(*regions)[index_region].Kx.push_back(temp_Kx[it_col_kx*(*control_param).number_controls_actions + it_row_kx]);
			}
		}

        //Reading Kc from .mat
		double *pointer_Kc = mxGetPr(element_region[3 * (*control_param).total_regions + index_region]);
		(*regions)[index_region].Kc.assign(pointer_Kc, pointer_Kc + (*control_param).number_controls_actions);
    }
	
    matClose(pmat_regions);
}
