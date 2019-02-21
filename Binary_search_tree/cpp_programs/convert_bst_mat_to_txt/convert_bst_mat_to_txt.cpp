#include "C:\Users\schul\Documents\UDESC\TCC\TCC1\Programas MATLAB\GitHub\Explicit-MPC\Binary_search_tree\cpp_programs\convert_bst_mat_to_txt\convert_bst_mat_to_txt.h"
#include "C:\Users\schul\Documents\UDESC\TCC\TCC1\Programas MATLAB\GitHub\Explicit-MPC\Binary_search_tree\cpp_programs\explicit_mpc_bst\explicit_mpc_bst.h"
#include "C:\Users\schul\Documents\UDESC\TCC\TCC1\Programas MATLAB\GitHub\Explicit-MPC\Binary_search_tree\cpp_programs\convert_bst_mat_to_txt\mat.h"
#include <iostream>
#include <vector>
#include <fstream>

void write_double_to_file(double *element, std::ofstream *file) {
	(*file).write((char*)&(*element), sizeof(*element));
}

void write_int_to_file(int *element, std::ofstream *file) {
	(*file).write((char*)&(*element), sizeof(*element));
}

void export_regions_to_file(struct_control_param *control_param, std::vector<struct_regions> *regions, const char *filename) {

	std::ofstream file_regions(filename, std::ios::out | std::ios::binary);

	//Export total number of regions
	write_int_to_file(&((*control_param).total_regions), &file_regions);

	//Export number of states
	write_int_to_file(&((*control_param).number_states), &file_regions);

	//Export number of controls actions u
	write_int_to_file(&((*control_param).number_controls_actions), &file_regions);

	for (int index_region = 0; index_region < (*control_param).total_regions; index_region++) {
		//Export Kx
		for (int it_kx = 0; it_kx < (*control_param).number_controls_actions * (*control_param).number_states; it_kx++) {
			double element_kx = (*regions)[index_region].Kx[it_kx];
			write_double_to_file(&element_kx, &file_regions);
		}

		//Export Kc
		for (int it_kc = 0; it_kc < (*control_param).number_controls_actions; it_kc++) {
			double element_kc = (*regions)[index_region].Kc[it_kc];
			write_double_to_file(&element_kc, &file_regions);
		}
	}
	file_regions.close();
}

void export_bst_to_file(struct_control_param *control_param, std::vector<struct_bst> *bst_nodes , const char *filename){

    std::ofstream file_bst(filename, std::ios::out | std::ios::binary);

	//Export total number of regions
	write_int_to_file(&((*control_param).total_nodes), &file_bst);

	//Export number of states
	write_int_to_file(&((*control_param).number_states), &file_bst);

	for (int index_node = 0; index_node < (*control_param).total_nodes; index_node++){

        //Export Inequation Index
		write_int_to_file(&((*bst_nodes)[index_node].ineq_index), &file_bst);

        //Export left
        write_int_to_file(&((*bst_nodes)[index_node].left),&file_bst);

        //Export right
        write_int_to_file(&((*bst_nodes)[index_node].right),&file_bst);

		//Export parent node
		write_int_to_file(&((*bst_nodes)[index_node].parent_node), &file_bst);

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

void export_ineq_set_to_file(struct_control_param *control_param, std::vector<struct_ineq_set> *ineq_set, const char *filename) {

	std::ofstream file_ineq_set(filename, std::ios::out | std::ios::binary);

	//Export total number of inequations
	write_int_to_file(&((*control_param).total_ineq), &file_ineq_set);

	//Export number of states
	write_int_to_file(&((*control_param).number_states), &file_ineq_set);

	for (int index_ineq = 0;  index_ineq < (*control_param).total_ineq;  index_ineq++) {
		//Export A
		for (int it_A = 0; it_A <(*control_param).number_states; it_A++) {
			double element_A;
			element_A = (*ineq_set)[index_ineq].A[it_A];
			write_double_to_file(&element_A, &file_ineq_set);
		}
		//Export b
		write_double_to_file(&((*ineq_set)[index_ineq].b), &file_ineq_set);
	}
	file_ineq_set.close();
}

void export_control_param_to_file(struct_control_param *control_param, const char *filename) {

	std::ofstream file_control_param(filename, std::ios::out | std::ios::binary);

	//Export number of states
	write_int_to_file(&((*control_param).number_states), &file_control_param);

	//Export number of control actions
	write_int_to_file(&((*control_param).number_controls_actions), &file_control_param);

	//Export total number of regions
	write_int_to_file(&((*control_param).total_regions), &file_control_param);

	//Export total number of nodes
	write_int_to_file(&((*control_param).total_nodes), &file_control_param);

	//Export total number of inequations
	write_int_to_file(&((*control_param).total_ineq), &file_control_param);

	//Export number of fields from bst
	write_int_to_file(&((*control_param).number_fields_bst), &file_control_param);

	//Export number of fields from regions
	write_int_to_file(&((*control_param).number_fields_regions), &file_control_param);

	//Export number of fields from ineq
	write_int_to_file(&((*control_param).number_fields_ineq), &file_control_param);
}

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

void get_struct_ineq_from_mat(struct_control_param *control_param, std::vector<struct_ineq_set> *ineq_set, const char *filename) {

	MATFile *pmat_ineq;

	pmat_ineq = matOpen(filename, "r");
	//Get struct from the .mat, it is maybe necessary to change the struct name
	mxArray *mat_data_ineq = matGetVariable(pmat_ineq, "ineq_pos");

	(*control_param).total_ineq = mxGetM(mat_data_ineq);//mxGetNumberOfElements(mat_struct_regions);
	(*control_param).number_fields_ineq = mxGetN(mat_data_ineq);

	mxArray **element_ineq;
	element_ineq = (mxArray **)mxGetData(mat_data_ineq);

	for (int index_ineq = 0; index_ineq < (*control_param).total_ineq; index_ineq++) {
		(*ineq_set).push_back(struct_ineq_set());

		//Reading A inequation from .mat
		double *A_ineq = mxGetPr(element_ineq[index_ineq]);
		(*ineq_set)[index_ineq].A.assign(A_ineq, A_ineq + (*control_param).number_states);
		
		//Reading b inequation from .mat
		(*ineq_set)[index_ineq].b = *(mxGetPr(element_ineq[1 * (*control_param).total_ineq + index_ineq]));
	}


	matClose(pmat_ineq);
}