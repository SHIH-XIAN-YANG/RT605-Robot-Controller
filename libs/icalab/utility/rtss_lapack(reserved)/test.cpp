#include<windows.h>

#include<iostream>
#include<numeric>

#define a1 1000
#define a2 1000
#define a3 1000
#define a4 1000000

//#include"rtss_utility"
void native_matmul(double* m1, double* m2, double* c){
	// c(m, n) = m1(m, k)*m2(k, n)
	unsigned int m{a1}, n{a2}, k{a3};
	for(unsigned int i = 0; i<m; ++i)
		for(unsigned int j = 0; j<n; ++j)
			for(unsigned int p = 0; p<k; ++p)
				c[i*n + j] += m1[i*n + p]*m2[p*n + j];
}
void inner_matmul(double* m1, double* m2, double* c){
	// c(m, n) = m1(m, k)*m2(k, n)
	unsigned int m{a1}, n{a2}, k{a3};
		for(unsigned int i = 0; i<m; ++i)
			for(unsigned int j = 0; j<n; ++j)
				c[i*n+j] = std::inner_product(m1+(i*n), m1+(i*n+n),m2+(i*n), 0.0); 
}

double m1[a4];
double m2[a4];
double m3[a4];

int main(int argv, char* argc[]){
/*
	icalab::Matrix<float> mat1(3, 3);
	for(unsigned int i = 0; i<9; ++i)
		*(mat1.m_pData + i) = i+1;
	std::cout<< mat1.RowNum() << "\t" << mat1.ColNum() << std::endl;
	//mat1.Print();
	std::cout << mat1(1, 1) + mat1(2, 2) << std::endl;
*/
	std::cout << "statr" << std::endl;
	

	
	LARGE_INTEGER freq;
	QueryPerformanceFrequency(&freq);
	LARGE_INTEGER t1;
	LARGE_INTEGER t2;
	
	QueryPerformanceCounter(&t1);
	//native_matmul(m1, m2, m3);
	Sleep(1);
	QueryPerformanceCounter(&t2);
	long long elasped = ( ((t2.QuadPart - t1.QuadPart) * 1000)/freq.QuadPart);
	std::cout << "Elasped time for Native MatMul: " << ((double)elasped)/1000.0 << std::endl;
	
	QueryPerformanceCounter(&t1);
	native_matmul(m1, m2, m3);
	QueryPerformanceCounter(&t2);
	elasped = ( ((t2.QuadPart - t1.QuadPart) * 1000)/freq.QuadPart);
	std::cout << "Elasped time for Native MatMul: " << ((double)elasped)/1000.0 << std::endl;
	
	//Sleep(1000);
	
	QueryPerformanceCounter(&t1);
	inner_matmul(m1, m2, m3);
	QueryPerformanceCounter(&t2);
	elasped = (((t2.QuadPart - t1.QuadPart) * 1000)/freq.QuadPart);
	std::cout << "Elasped time for Inner-product MatMul: " << ((double)elasped)/1000.0 << std::endl;

	std::cout << "end" << std::endl;
	return 0;
}