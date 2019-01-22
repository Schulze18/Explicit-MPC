#include "explicit_mpc_bst.h"

int total_nodes;
int total_regions;
int number_states;
int number_controls_actions;

void read_double_from_file(double *element, std::ifstream *file) {
	(*file).read((char*)&(*element), sizeof(*element));
}

void read_int_from_file(int *element, std::ifstream *file) {
	(*file).read((char*)&(*element), sizeof(*element));
}

void get_bst_from_file(std::vector<struct_bst> *bst_nodes, const char *filename) {

	std::ifstream file_bst(filename, std::ios::in | std::ios::binary);

	read_int_from_file(&total_nodes, &file_bst);
	read_int_from_file(&number_states, &file_bst);

	for (int index_node = 0; index_node < total_nodes; index_node++){
        (*bst_nodes).push_back(struct_bst());

        //Get A
        for (int number_A = 0; number_A < number_states; number_A++){
            double element_A;
            read_double_from_file(&element_A,&file_bst);
			(*bst_nodes)[index_node].A.push_back(element_A);
		}

		//Get b
		read_double_from_file(&((*bst_nodes)[index_node].b),&file_bst);

        //Get left
        read_int_from_file(&((*bst_nodes)[index_node].left),&file_bst);

		//Get right
        read_int_from_file(&((*bst_nodes)[index_node].right),&file_bst);

        //Get parent_node
        read_int_from_file(&((*bst_nodes)[index_node].parent_node),&file_bst);

        //Get Number of regions
        int number_regions;
        read_int_from_file(&number_regions,&file_bst);

        //Get Regions
        for (int it_region = 0; it_region < number_regions; it_region++){
            int element_region;
            read_int_from_file(&element_region,&file_bst);
            (*bst_nodes)[index_node].regions.push_back(element_region);
        }
	}

	file_bst.close();
}

void get_regions_from_file(std::vector<struct_regions> *regions, const char *filename){

    std::ifstream file_regions(filename, std::ios::in | std::ios::binary);

    read_int_from_file(&total_regions, &file_regions);
	read_int_from_file(&number_controls_actions, &file_regions);

    for (int index_region = 0; index_region < total_regions; index_region++) {
		(*regions).push_back(struct_regions());

        //Ger Inequation number
        int number_inequation;
        read_int_from_file(&number_inequation, &file_regions);
        printf("region %d: %d ineqs\n", index_region, number_inequation);

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

        //Get Kx
		for (int it_kx = 0; it_kx < number_controls_actions*number_states; it_kx++) {
			double element_kx;

			read_double_from_file(&element_kx, &file_regions);
			(*regions)[index_region].Kx.push_back(element_kx);
		}

        //Get Kc
		for (int it_kc = 0; it_kc < number_controls_actions; it_kc++) {
			double element_kc;

			read_double_from_file(&element_kc,&file_regions);
			(*regions)[index_region].Kc.push_back(element_kc);
		}
        printf("region %d   kx first: %lf  second: %lf  third: %lf\n", index_region, (*regions)[index_region].Kx[0], (*regions)[index_region].Kx[1], (*regions)[index_region].Kx[2]);
    }
    file_regions.close();
}


