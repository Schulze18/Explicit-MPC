#include <iostream>
#include "convert_bst_mat_to_txt.h"

using namespace std;

int main()
{
    std::vector<struct_bst> bst_nodes;
    std::vector<struct_regions> regions;

    get_struct_bst_from_mat(&bst_nodes , "struct_bst_nodes_10_3_u_du_v2.mat");
    get_struct_regions_from_mat(&regions, "struct_regions_10_3_u_du_v2.mat");

    export_bst_to_file(&bst_nodes , "output_bst_nodes.txt");
    export_regions_to_file(&regions, "output_regions.txt");

    return 0;
}
