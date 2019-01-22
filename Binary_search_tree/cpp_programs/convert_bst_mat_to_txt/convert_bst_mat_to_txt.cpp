#include "convert_bst_mat_to_txt.h"

int number_fields_bst;
int number_fields_regions;
int max_number_ineq;

void write_double_to_file(double *element, std::ofstream *file) {
	(*file).write((char*)&(*element), sizeof(*element));
}

void write_int_to_file(int *element, std::ofstream *file) {
	(*file).write((char*)&(*element), sizeof(*element));
}

void export_bst_to_file(std::vector<struct_bst> *bst_nodes , const char *filename){

    std::ofstream file_bst(filename, ios::out | ios::binary);

    write_int_to_file(&total_nodes,&file_bst);
    write_int_to_file(&number_states,&file_bst);

	for (int index_node = 0; index_node < total_nodes; index_node++){

        //Export A
        for (int it_A = 0; it_A < number_states; it_A++){
            double element_A;
			element_A = (*bst_nodes)[index_node].A[number_A];
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
        for (int it_region = 0; it_region < number_regions; it_region++){
            int element_region = (*bst_nodes)[index_node].regions[it_region];
                write_int_to_file(&element_region,&file_bst);
        }
	}
    file_bst.close();

}

void export_regions_to_file(std::vector<struct_regions> *regions, const char *filename){

    std::ofstream file_regions(filename, ios::out | ios::binary);

    //Export total number of regions
    write_int_to_file(&total_regions,&file_regions);

    //Export number of controls actions u
    write_int_to_file(&number_controls_actions, &file_regions);

    for (int index_region = 0; index_region < total_regions; index_region++) {

        //Export number of inequation that define the region
        int number_inequation_region = (*regions)[index_region].set_A.size();
        write_int_to_file(&number_inequation_region,&file_regions);

        //Export inequations set A
        for (int it_ineq = 0; it_ineq < number_inequation; it_ineq++) {
			for (int it_element = 0; it_element < number_states; it_element++) {
                double element_ineq_A = (*regions)[index_region].set_A[it_ineq][it_element];
                write_double_to_file(&element_ineq_A,&file_regions);
            }
        }

        //Export b
        for (int it_ineq = 0; it_ineq < number_inequation; it_ineq++) {
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
}

void get_struct_bst_from_mat(std::vector<struct_bst> *bst_nodes , const char *filename){
    MATFile *pmat_bst;

    pmat = matOpen(filename, "r");/*
    //Get struct from the .mat file
	mxArray *struct_node = matGetVariable(pmat, "struct_nodes");

	//Get all the data from the struct
	mxArray **data_elements;
	data_elements = (mxArray **)mxGetData(struct_node);

    //int total_nodes;
   //const char *filename2 = "output_bst_nodes.txt";
    //std::ifstream file_bst(filename2, std::ios::in | std::ios::binary);
    //read_int_from_file(&total_nodes, &file_bst);
	number_fields_bst = mxGetNumberOfFields(struct_node);
	int total_nodes = mxGetNumberOfElements(struct_node);
    int number_states = mxGetN(data_elements[0]);

    printf("test number fields %d\n", number_fields_bst );//  total nodes %d   number states: %d\n", number_fields_bst, total_nodes, number_states);
*/

    pmat_bst = matOpen(filename, "r");
	//Get struct from the .mat, it is maybe necessary to change the struct name
	mxArray *struct_nodes = matGetVariable(pmat, "struct_nodes");

    //Get all the data from the struct
	mxArray **bst_elements;
	bst_elements = (mxArray **)mxGetData(struct_nodes);

    number_fields_bst = mxGetNumberOfFields(struct_nodes);
	total_nodes = mxGetNumberOfElements(struct_nodes);
	number_states = mxGetN(bst_elements[0]);

    for (int index_node = 0; index_node < total_nodes; index_node++) {
        (*bst_nodes).push_back(struct_bst());

        //Reading A inequations from .mat
        double *A_ineq = mxGetPr(bst_elements[0 + index_node * number_fields]);
		(*bst_nodes)[index_node].A.assign(A_ineq, A_ineq + number_states);

        //Reading b from .mat
        (*bst_nodes)[index_node].b = *(mxGetPr(bst_elements[1 + index_node * number_fields_bst]));

        //Reading left node from .mat
		(*bst_nodes)[index_node].left = *(mxGetPr(bst_elements[2 + index_node * number_fields_bst]));

		//Reading right node from .mat
		(*bst_nodes)[index_node].right = *(mxGetPr(bst_elements[3 + index_node * number_fields_bst]));

		//Reading parent node from .mat
		(*bst_nodes)[index_node].parent_node = *(mxGetPr(bst_elements[4 + index_node * number_fields_bst]));

		//Reading all the regions nodes
		double *regions_node = mxGetPr(bst_elements[5 + index_node * number_fields_bst]);
		int max_number_regions = mxGetM(bst_elements[5 + index_node * number_fields_bst]);
		std::vector<int> temp_regions(regions_node, regions_node + max_number_regions);

		//Logic to not get null index
		for (int it_region = 0; it_region < max_number_regions; it_region++) {
			if (temp_regions[it_region] != 0){
				(*bst_nodes)[index_node].regions.push_back(temp_regions[it_region]);
			}
			else break;
		}
    }
    matClose(pmat_bst);
}


void get_struct_regions_from_mat(std::vector<struct_regions> *regions, const char *filename){
    MATFile *pmat_regions;

    pmat_regions = matOpen(filename, "r");
    //Get struct from the .mat, it is maybe necessary to change the struct name
    mxArray *mat_struct_regions = matGetVariable(pmat_regions, "reg_struct");

	mxArray **element_region;
	element_region = (mxArray **)mxGetData(mat_struct_regions);

	number_fields_regions = mxGetNumberOfFields(mat_struct_regions);
	total_regions = mxGetNumberOfElements(mat_struct_regions);
	max_number_ineq = mxGetM(element_region[0]);
	number_controls_actions = mxGetM(element_region[2]);

    for (int index_region = 0; index_region < total_regions; index_region++) {
        (*regions).push_back(struct_regions());

		//Reading A inequations from .mat
		double *pointer_set_A_ineq = mxGetPr(element_region[0 + index_region * number_fields_regions]);
		std::vector<double> set_A_ineq;
		set_A_ineq.assign(pointer_set_A_ineq, pointer_set_A_ineq + number_states*max_number_ineq);

		//Logic to not include null rows
		for (int it_ineq = 0; it_ineq < max_number_ineq; it_ineq++) {
			int flag_zero_ineq = 1;
			std::vector<double> temp_inequation;

			for (int it_element = 0; it_element < number_states; it_element++) {
				temp_inequation.push_back(set_A_ineq[it_element*max_number_ineq + it_ineq]);
				if (set_A_ineq[it_element*max_number_ineq + it_ineq] != 0) {
                    flag_zero_ineq = 0;
				}
			}

			if (flag_zero_ineq == 0) {
				(*regions)[index_region].set_A.push_back(temp_inequation);
            }
		}

		int number_ineq_A = regions[index_region].set_A.size();
		//printf("size A region %d: %d\n", index_region+1, number_inequation);

		//Reading b from .mat
		double *pointer_set_b_ineq = (mxGetPr(element_region[1 + index_region * number_fields_regions]));
		(*regions)[index_region].set_b.assign(pointer_set_b_ineq,pointer_set_b_ineq + number_ineq_A);

		//Reading Kx from .mat
		double *pointer_Kx = mxGetPr(element_region[2 + index_region * number_fields_regions]);
		vector<double> temp_Kx;
		temp_Kx.assign(pointer_Kx, pointer_Kx + number_controls_actions * number_states);

		//Logic to sort the elements in the desired order
		for (int it_row_kx = 0; it_row_kx < number_controls_actions; it_row_kx++) {
			for (int it_col_kx = 0; it_col_kx < number_states; it_col_kx++) {
				(*regions)[index_region].Kx.push_back(temp_Kx[it_col_kx*number_controls_actions + it_row_kx]);
			}
		}

        //Reading Kc from .mat
		double *pointer_Kc = mxGetPr(element_region[3 + index_region * number_fields_regions]);
		regions[index_region].Kc.assign(pointer_Kc, pointer_Kc + number_controls_actions);
    }
    matClose(pmat_regions);
}
