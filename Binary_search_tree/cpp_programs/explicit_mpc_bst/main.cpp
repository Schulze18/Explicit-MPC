#include <iostream>
#include "explicit_mpc_bst.h"

using namespace std;

int main() {
	std::vector<struct_bst> bst_nodes;
    std::vector<struct_regions> regions;

	get_bst_from_file(&bst_nodes, "output_bst_nodes.txt");

    get_regions_from_file(&regions, "output_regions.txt");

   // printf("test size nodes: %d \n", bst_nodes.size());
   // printf("test size: %d \n", bst_nodes[0].regions.size());
   // printf("test size: %d \n", bst_nodes[55].regions.size());

    double state_x[] = {3, 0.1, -0.7, 0, 6, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0};
    //state_x = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int index_region = index_region_evaluate_bst(state_x, &bst_nodes);
    cout << index_region << endl;

    double *control_action;
    control_action = calculate_control_explicit_bst(state_x, &regions, index_region - 1);

   /* for (int it_control = 0; it_control <  4; it_control++)
    {
       // cout << *(control_action + it_control) << endl;
    }*/

	return 0;
}
