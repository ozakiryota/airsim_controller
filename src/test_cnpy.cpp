#include <iostream>
#include"cnpy.h"

int main(void)
{
	int size_h = 5;
	int size_w = 10; 

	std::vector<int> mat;
	mat.resize(size_h*size_w);

	for(int i=0; i<size_h*size_w; ++i){
		mat[i] = i;
	}   

	std::string save_path = "test_cnpy.npy";
	cnpy::npy_save(save_path, &mat[0], {size_h, size_w}, "w");
	std::cout << "save_path = " << save_path << std::endl;
}
