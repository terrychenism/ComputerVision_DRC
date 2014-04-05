#include <iostream>
#include <fstream>
int main(int argc, char * argv[])
{
	std::fstream myfile("aaa.txt", std::ios_base::in);
	int width = 924;
	int height = 924;
	double a;
	double* array;
	array = new double[width*height* 3];
	int i = 0;
	while (myfile >> a){
		array[i] = a;
		i++;
	}
		
	std::cout << array[width * height * 3 - 1] << std::endl;
	getchar();
	return 0;
}
