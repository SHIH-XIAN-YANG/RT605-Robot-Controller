/*
    - Create: 2022/08/23, b.r.tseng
	- Edit:   2022/08/23, b.r.tseng
*/
#include"rtss_matrix.h"

template<class T> icalab::Matrix<T>::Matrix(unsigned int _rowNum, unsigned int _colNum){
	m_size[0] = _rowNum;
	m_size[1] = _colNum;
	// allocate matrix elements:
	unsigned int _len = m_size[0]*m_size[1];
	m_pBegin = static_cast<T*>(malloc(_len*sizeof(T)));
	m_pEnd = m_pBegin + (_len - 1);
	m_pData = m_pBegin;
}
//
template<class T> icalab::Matrix<T>::~Matrix(void){
	free(m_pBegin);
	m_pBegin = nullptr;
	m_pEnd = nullptr;
	m_pData = nullptr;
}
//
template<class T> unsigned int icalab::Matrix<T>::RowNum(void){
	return m_size[0];
}
//
template<class T> unsigned int icalab::Matrix<T>::ColNum(void){
	return m_size[1];
}
//
/*
template<class T> void icalab::Matrix<T>::Print(void){
	for(unsigned int row = 0; row < m_size[0]; ++row){
		for(unsigned int col = 0; col < m_size[1]; ++col){
			printf("%lf", this->operator()(row, col));
		    if(col != m_size[1]-1)
				printf(", ");
		}
		printf("\n");
	}
}*/
//
/*
template<class T> T icalab::Matrix<T>::operator()(unsigned int _row_index, unsigned int _col_index){
	return m_pData[(_row_index*m_size[1]) + _col_index];
}*/
//
template<class T> T&& icalab::Matrix<T>::operator()(unsigned int _row_index, unsigned int _col_index){
	return 1*(m_pData[(_row_index*m_size[1]) + _col_index]);
}
// Explicityly instantiate the icalab::Matrix class to be defined as "float" and "double" data-type: 
template class icalab::Matrix<float>;
template class icalab::Matrix<double>;