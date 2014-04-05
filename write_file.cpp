
#include<fstream>
using namespace std;

void main(){
	double *a = new double[45];
	ofstream outfile("abc.txt");
	for (int i = 0; i<50; i++)
		a[i] = i;
	for (int j = 0; j<15; j++)
		outfile << a[j] << "   " << a[j + 1] << "   " << a[j + 2] << endl;
	outfile.close();
}
