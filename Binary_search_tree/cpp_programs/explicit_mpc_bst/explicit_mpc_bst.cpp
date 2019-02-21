#include "C:\Users\schul\Documents\UDESC\TCC\TCC1\Programas MATLAB\GitHub\Explicit-MPC\Binary_search_tree\cpp_programs\explicit_mpc_bst\explicit_mpc_bst.h"
#include "C:\Users\schul\Documents\UDESC\TCC\TCC1\Programas MATLAB\GitHub\Explicit-MPC\Binary_search_tree\cpp_programs\explicit_mpc_bst\matrix_drque.h"
#include <iostream>
#include <vector>

void read_double_from_file(double *element, std::ifstream *file) {
	(*file).read((char*)&(*element), sizeof(*element));
}

void read_int_from_file(int *element, std::ifstream *file) {
	(*file).read((char*)&(*element), sizeof(*element));
}

void get_regions_from_file(std::vector<struct_regions> *regions, const char *filename) {

	std::ifstream file_regions(filename, std::ios::in | std::ios::binary);

	int total_regions;
	int number_states;
	int number_controls_actions;

	read_int_from_file(&total_regions, &file_regions);
	read_int_from_file(&number_states, &file_regions);
	read_int_from_file(&number_controls_actions, &file_regions);

	for (int index_region = 0; index_region < total_regions; index_region++) {
		(*regions).push_back(struct_regions());
		/*
		//Ger Inequation number
		int number_inequation;
		read_int_from_file(&number_inequation, &file_regions);
		//printf("region %d: %d ineqs\n", index_region, number_inequation);

		//Get A
		for (int it_ineq = 0; it_ineq < number_inequation; it_ineq++) {
			std::vector<double> temp_ineq_A;

			for (int it_element = 0; it_element < number_states; it_element++) {
				double element_ineq_A;

				read_double_from_file(&element_ineq_A, &file_regions);
				temp_ineq_A.push_back(element_ineq_A);
			}

			(*regions)[index_region].set_A.push_back(temp_ineq_A);
		}


		//Get b
		for (int it_ineq = 0; it_ineq < number_inequation; it_ineq++) {
			double element_b;

			read_double_from_file(&element_b, &file_regions);
			(*regions)[index_region].set_b.push_back(element_b);
		}
		*/
		//Get Kx
		for (int it_kx = 0; it_kx < number_controls_actions*number_states; it_kx++) {
			double element_kx;

			read_double_from_file(&element_kx, &file_regions);
			(*regions)[index_region].Kx.push_back(element_kx);
		}

		//Get Kc
		for (int it_kc = 0; it_kc < number_controls_actions; it_kc++) {
			double element_kc;

			read_double_from_file(&element_kc, &file_regions);
			(*regions)[index_region].Kc.push_back(element_kc);
		}
		// printf("region %d   kx first: %lf  second: %lf  third: %lf\n", index_region, (*regions)[index_region].Kx[0], (*regions)[index_region].Kx[1], (*regions)[index_region].Kx[2]);
	}
	file_regions.close();
}

void get_bst_from_file(std::vector<struct_bst> *bst_nodes, const char *filename) {

	std::ifstream file_bst(filename, std::ios::in | std::ios::binary);

	int total_nodes;
	int number_states;

	read_int_from_file(&total_nodes, &file_bst);
	read_int_from_file(&number_states, &file_bst);

	for (int index_node = 0; index_node < total_nodes; index_node++){
        (*bst_nodes).push_back(struct_bst());
		/*
        //Get A
        for (int it_A = 0; it_A < number_states; it_A++){
            double element_A;
            read_double_from_file(&element_A,&file_bst);
			(*bst_nodes)[index_node].A.push_back(element_A);
		}

		//Get b
		read_double_from_file(&((*bst_nodes)[index_node].b),&file_bst);*/

		//Get Inequation Index
		read_int_from_file(&((*bst_nodes)[index_node].ineq_index), &file_bst);

        //Get left
        read_int_from_file(&((*bst_nodes)[index_node].left),&file_bst);

		//Get right
        read_int_from_file(&((*bst_nodes)[index_node].right),&file_bst);

        //Get parent_node
        read_int_from_file(&((*bst_nodes)[index_node].parent_node),&file_bst);

        //Get Number of regions
        int number_regions_node;
        read_int_from_file(&number_regions_node, &file_bst);

        //Get Regions
        for (int it_region = 0; it_region < number_regions_node; it_region++){
            int element_region;
            read_int_from_file(&element_region,&file_bst);
            (*bst_nodes)[index_node].regions.push_back(element_region);
        }
	}
	file_bst.close();
}

void get_ineq_set_from_file(std::vector<struct_ineq_set> *ineq_set, const char *filename) {

	std::ifstream file_ineq_set(filename, std::ios::in | std::ios::binary);

	int total_ineq;
	int number_states;

	read_int_from_file(&total_ineq, &file_ineq_set);
	read_int_from_file(&number_states, &file_ineq_set);

	for (int index_ineq = 0; index_ineq < total_ineq; index_ineq++) {
		(*ineq_set).push_back(struct_ineq_set());

		//Get A
		for (int it_A = 0; it_A < number_states; it_A++) {
			double element_A;
			read_double_from_file(&element_A, &file_ineq_set);
			(*ineq_set)[index_ineq].A.push_back(element_A);
		}

		//Get b
		read_double_from_file(&((*ineq_set)[index_ineq].b), &file_ineq_set);
	}
	file_ineq_set.close();
}

void get_control_param_file(struct_control_param *control_param, const char *filename) {

	std::ifstream file_control_param(filename, std::ios::in | std::ios::binary);

	//Get number of states
	read_int_from_file(&((*control_param).number_states), &file_control_param);

	//Get number of control actions
	read_int_from_file(&((*control_param).number_controls_actions), &file_control_param);

	//Get total number of regions
	read_int_from_file(&((*control_param).total_regions), &file_control_param);

	//Get total number of nodes
	read_int_from_file(&((*control_param).total_nodes), &file_control_param);

	//Get total number of inequations
	read_int_from_file(&((*control_param).total_ineq), &file_control_param);

	//Get number of fields from bst
	read_int_from_file(&((*control_param).number_fields_bst), &file_control_param);

	//Get number of fields from regions
	read_int_from_file(&((*control_param).number_fields_regions), &file_control_param);

	//Get number of fields from ineq
	read_int_from_file(&((*control_param).number_fields_ineq), &file_control_param);
}


//int index_region_evaluate_bst(double state[], std::vector<struct_bst> *bst_nodes, std::vector<struct_bst>){
int index_region_evaluate_bst(double state[], size_t number_states, std::vector<struct_bst> *bst_nodes, std::vector<struct_ineq_set> *ineq_set){

    int index_node = 0;
    //Matrix <double> test;

    while((*bst_nodes)[index_node].left != -1){
		int index_ineq = (*bst_nodes)[index_node].ineq_index - 1;

        //size_t number_elements = (*bst_nodes)[index_node].A.size();
        //double A_temp[number_elements];
        //std::copy((*bst_nodes)[index_node].A.begin(), (*bst_nodes)[index_node].A.end(), A_temp);
		
		double *A_temp = &((*ineq_set)[index_ineq].A[0]);

        Matrix <double> A_ineq(1,number_states);
        A_ineq = A_temp;
        Matrix <double> matrix_state(number_states,1);
        matrix_state = state;
        Matrix <double> matrix_product = A_ineq*matrix_state;

		
		if((matrix_product.get(0, 0) - (*ineq_set)[index_ineq].b) <= 0)index_node = ((*bst_nodes)[index_node].left - 1);
		else index_node = ((*bst_nodes)[index_node].right - 1);
	}
    return (*bst_nodes)[index_node].regions[0];
}

void calculate_control_explicit_bst(double *array_control_action, double state[], size_t number_states, size_t number_controls_actions, std::vector<struct_regions> *regions, int index_region){

    ///double kx_temp[(*regions)[index_region].Kx.size()];
    //double kx_temp[number_controls_actions*number_states];
    //std::copy((*regions)[index_region].Kx.begin(), (*regions)[index_region].Kx.end(), kx_temp);

	double *kx_temp = &((*regions)[index_region].Kx[0]);

    Matrix <double> Kx(number_controls_actions, number_states);
    Kx = kx_temp;

    //double kc_temp[number_controls_actions];
    //double kc_temp[(*regions)[index_region].Kc.size()];
    //std::copy((*regions)[index_region].Kc.begin(), (*regions)[index_region].Kc.end(), kc_temp);
    
	
	double *kc_temp = &((*regions)[index_region].Kc[0]);
	Matrix <double> Kc(number_controls_actions,1);
    Kc = kc_temp;

    Matrix <double> matrix_state(number_states,1);
    matrix_state = state;

    Matrix <double> matrix_control_action = Kx*matrix_state + Kc;

    for (int it_control = 0; it_control < number_controls_actions; it_control++){
        array_control_action[it_control] = matrix_control_action.get(it_control,0);
        //printf("%lf\n", array_control_action[it_control]);
    }
}
