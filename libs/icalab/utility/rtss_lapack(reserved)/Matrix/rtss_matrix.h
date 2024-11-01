/*
    - Create: 2022/08/23, b.r.tseng
	- Edit:   2022/08/23, b.r.tseng
*/
#ifndef __RTSS_MATRIX_H__
#define __RTSS_MATRIX_H__
// std C:
#include<Windows.h>
#include<string.h>
#include<stdlib.h>
#include<stdio.h>
// std C++:
#include<utility> // for std::move

namespace icalab{
	template<class T>
	class Matrix{
		protected:
			unsigned int m_size[2];
			T* m_pBegin;
			T* m_pEnd;
			
		public:
			T* m_pData;
			Matrix(unsigned int _rowNum, unsigned int _colNum);
			~Matrix(void);
			unsigned int RowNum(void);
			unsigned int ColNum(void);
			//void Print(void); 
			//T 	operator ()(unsigned int _row_index, unsigned int _col_index);
			T&& operator ()(unsigned int _row_index, unsigned int _col_index);
			//T   operator * (Matrix<T>& matB);
			//T   operator * (Matrix<T>&& matB);
			//T&& operator * (Matrix<T>& matB);
			
	};
}

#endif