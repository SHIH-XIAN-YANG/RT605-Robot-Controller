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
#define      RELEASE_VECTOR(vec) std::vector<std::vector<float>>().swap(vec);



inline bool Read_CSP_program(const char* filename, std::vector<std::vector<float>>& csp_buf) {
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
        printf(" - The csp points hadn't been import, completely.\n");
    fs.close();
    // Get row content of the file into the designated string object (tmpRowData): 
    int index{ 0 };
    // Convert the read buffer data into the input-string-stream:
    RowSStream.str(temp_Buf);
    std::vector<float> temp_pointsVec(6, 0.0);
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
inline unsigned int Compute_CSP_point(const char* filename) {
    printf(" - Computeing the number of the interpolation point ....... \n");
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
        printf(" - Error: Failed to open the CSP program.\n");
    fs.close();
    // Get row content of the file into the designated string object (tmpRowData): 
    int index{ 0 };
    // Convert the read buffer data into the input-string-stream:
    RowSStream.str(temp_Buf);
    unsigned int length{ 0 };
    while (std::getline(RowSStream, tmpRowData, '\n')) ++length; // Calculate the number of the interpolation points.
    RtPrintf(" - The amout of interpolation point: %i.\n", length);
    return length;
}
inline bool Read_CSP_program(const char* filename, double** csp) {
    bool Ret;
    FILE* fs;
    fs = fopen(filename, "r");
    printf(" - Open the file stream: INTP Program -> Local Heap\n");
    unsigned int n{ 0 };
    char joint{ 0 };
    if (fs != nullptr) {
        while (!feof(fs)) {
            fscanf(fs, "%lf,%lf,%lf,%lf,%lf,%lf", &(csp[n][0]), &(csp[n][1]), &(csp[n][2]), &(csp[n][3]), &(csp[n][4]), &(csp[n][5]));
            ++n;
        }
        fclose(fs);
        RtPrintf(" - Point 0: %d, %d, %d, %d, %d, %d", (unsigned int)csp[0][0], (unsigned int)csp[0][1], (unsigned int)csp[0][2], (unsigned int)csp[0][3], (unsigned int)csp[0][4], (unsigned int)csp[0][5]);
        RtPrintf(" - Close the file stream.\n");
        Ret = true;
    }
    else {
        RtPrintf(" - Cannot find the INTP program file.\n");
        Ret = false;
    }
    return Ret;
}
//
template<class T>
inline void AllocateINTP_Buffer(const char* program_name, unsigned int intp_length, T**& intp_buffer) {
    intp_length = Compute_CSP_point(program_name);
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
/*
//
struct CSP_Attribute{
	double feedrate;
	double accelerate_time_ms;
	double decelerate_time_ms;
	double time;
	unsigned long long length;
};
//
template <class T>
class CSP_Buffer{
	private:
	const char* csp_program_name__m;
	T** csp_buffer__m;
	CSP_Attribute attribute__m;
    bool state__m; // false: empty, true: loaded
	// private function:
	bool Allocate_CSP_Buffer(void);
	bool Release_CSP_Buffer(void);
	public:
	// Constructor:
	CSP_Buffer(void);
	// public function:
	bool Read_CSP_program(const char* file_name);
	unsigned long long ComputePoint(const char* file_name);
	bool Print_CSP_Attribute(void);
	bool Print_CSP_Attribute(const char* file_name);
}
*/