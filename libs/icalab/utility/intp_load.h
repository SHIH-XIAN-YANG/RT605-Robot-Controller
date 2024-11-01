/*
*  - RT605-710 Robot Controller RTSS Real-time global objects
*  - Build    : 2021/01/06, B.R.Tseng
*  - Last Edit: 2022/03/24, B.R.Tseng
*/
#pragma once
#include<stdlib.h>
#include<string>
#include<fstream>
#include<sstream>
#include<vector>
#include<stdio.h>
#include<utility>
#define      RELEASE_VECTOR(vec) std::vector<std::vector<double>>().swap(vec);



inline bool Read_INTP_program(const char* filename, std::vector<std::vector<double>>& csp_buf) {
	if(csp_buf.size() > 1){
		RELEASE_VECTOR(csp_buf);
	}
    printf(" - Reading INTP program ...... \n");
    // Declare a std::string object where the extracted row data will be stored:
    std::string tmpRowData, tmpColData;
    std::istringstream RowSStream, ColSStream;
    // Construct file-stream:
    std::fstream fs(filename, std::fstream::in);
    // Compute interpolation length:
    fs.seekg(0, fs.end);
    unsigned int len = fs.tellg();
    fs.seekg(0, fs.beg);
    // Open the file-stream to read the file:
    char* temp_Buf = new char[len];
    fs.read(temp_Buf, len);
    if (!fs)
        printf(" - The INTP points hadn't been import, completely.\n");
    fs.close();
    // Get row content of the file into the designated string object (tmpRowData): 
    int index{ 0 };
    // Convert the read buffer data into the input-string-stream:
    RowSStream.str(temp_Buf);
    std::vector<double> temp_pointsVec(6, 0.0);
    while (std::getline(RowSStream, tmpRowData, '\n')) {
        int joint{ 0 };
        // Convert the tmpRowData into the input-string-stream:
        ColSStream.str(tmpRowData);
        // Split the row content by the delim ',':
        while (std::getline(ColSStream, tmpColData, ',')) {
            temp_pointsVec.at(joint++) = std::stof(tmpColData);
        }
        csp_buf.push_back(temp_pointsVec);
        ++index;
    }
    csp_buf.shrink_to_fit();
    printf(" - Interploation point: %i.\n", static_cast<unsigned int>(csp_buf.size()));
    delete[] temp_Buf; temp_Buf = nullptr;
    // Return code:
    return 1;
}
//
inline unsigned int Compute_INTP_point(const char* filename) {
	FILE* fs;
	fopen_s(&fs, filename, "r");
	double tmp[6];
   if (fs != nullptr) {
	 	while (!feof(fs)) {
			fscanf(fs, "%lf,%lf,%lf,%lf,%lf,%lf", tmp, tmp+1, tmp+2, tmp+3, tmp+4, tmp+5);
	 		++n;
	 	}
	 	fclose(fs);
	 	printf(" - The amout of interpolation point: %i.\n", length);
	 	return length;
	}
	else {
		printf(" - Cannot find the INTP program file.\n");
		return 0;
	}
}
inline bool Read_INTP_program(const char* filename, double** INTP) {
    bool Ret;
    FILE* fs;
    fs = fopen(filename, "r");
    printf(" - Open the file stream: INTP Program -> Local Heap\n");
    unsigned int n{ 0 };
    char joint{ 0 };
    if (fs != nullptr) {
        while (!feof(fs)) {
            fscanf(fs, "%lf,%lf,%lf,%lf,%lf,%lf", &(INTP[n][0]), &(INTP[n][1]), &(INTP[n][2]), &(INTP[n][3]), &(INTP[n][4]), &(INTP[n][5]));
            ++n;
        }
        fclose(fs);
        printf(" - Point 0: %d, %d, %d, %d, %d, %d", (unsigned int)INTP[0][0], (unsigned int)INTP[0][1], (unsigned int)INTP[0][2], (unsigned int)INTP[0][3], (unsigned int)INTP[0][4], (unsigned int)INTP[0][5]);
        printf(" - Close the file stream.\n");
        Ret = true;
    }
    else {
        printf(" - Cannot find the INTP program file.\n");
        Ret = false;
    }
    return Ret;
}
//
template<class T>
inline void AllocateINTP_Buffer(const char* program_name, unsigned int* intp_length, T**& intp_buffer) {
    *intp_length = Compute_INTP_point(program_name);
    if (intp_buffer != nullptr) {
        intp_buffer = (T**)realloc(intp_buffer, sizeof(T*) * intp_length);
        for (unsigned int i = 0; i < intp_length; ++i) {
            if (intp_buffer[i] == nullptr)
                intp_buffer[i] = (T*)calloc(6, sizeof(T));
        }
        printf(" - The INTP buffer has been re-allocated.\n");
    }
    else if (intp_buffer == nullptr) { // for the first time
        intp_buffer = (T**)calloc(intp_length, sizeof(T*));
        for (unsigned int i = 0; i < intp_length; ++i)
            intp_buffer[i] = (T*)calloc(6, sizeof(T));
        printf(" - The INTP buffer has been allocated.\n");
    }
}
/* Example1: (使用 std::vector 之版本)
	// (1) 宣告一個存放 intp 點位的暫存器之指標，與一個用來存放接下來要計算 intp 點位長度的變數:
		std::vector<std::vector<double>> intp_buf(6, std::vector<double>);
		unsigned int length = Compute_INTP_point("檔名.csv")
	// (2) 讀取：
		Read_INTP_program("檔名.csv", intp_buf);
*/

/* Example2: (使用 c 陣列之版本)
	// (1) 宣告一個存放 intp 點位的暫存器之指標，與一個用來存放接下來要計算 intp 點位長度的變數:
		double** intp_buf;
		unsigned int length;
	// (2) 自動計算要讀取的 intp 檔中差補點的長度，以及動態的分配 intp 暫存器之記憶體空間：
		AllocateINTP_Buffer<double>("檔名.csv", &length, intp_buf);
	// (3) 讀取：
		Read_INTP_program("檔名.csv", intp_buf);
			
*/