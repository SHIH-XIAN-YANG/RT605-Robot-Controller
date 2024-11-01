#ifndef __DATA_PROCESSING_H__
#define __DATA_PROCESSING_H__
#define COMPLETE true
#include <string>
#include <vector>
#include <utility> // for std::pair
#include <fstream>
#include"armadillo"

//typedef  dataframe
typedef arma::Col<int> dataI16; // int data vector
typedef arma::Col<unsigned int> dataU16; // unsigned int data vector
typedef arma::Col<double> dataDOU; // double data vector
typedef arma::Col<float> dataF8; // float data vector
typedef std::vector < std::pair < std::string, std::vector <double>> > datafile;
typedef std::vector<std::string> data_header;
typedef std::pair<std::string, std::vector<double>> dataset;
// -dataset: pair<(string)header name, (double)data value>
	//===============================================================================
	// Each column of data is represented by the pair <column name, column data>.
    // ex: 
	// (1) dataDOU data1_time = { 0,1,2,3,4,5,6,7,8,9,10 };
	//     dataDOU data1_y1 = { 10,11,12,13,14,15,16,17,18,19,20 };
	// (2) dataset data1 = {"time", data1_time};
    //     dataset data2 = {"y(t)", data1_y1};
    // (3) datafile datafile1 = {data1, data2};

// export a data 2020/07/08 -B.R.Tseng
bool export_datafile(std::string filename, datafile data) {
	// -filename
	// -dataset: pair<(string)header name, (double)data value>
	//===============================================================================
	// Each column of data is represented by the pair <column name, column data>.
	//===============================================================================
	// create and output filestream object
	std::ofstream file01(filename);
	// build a dataframe
	// send the data header names to the stream(dataframe)
	for (int header = 0; header < data.size(); ++header) {
		file01 << data.at(header).first; // the first element of the dataset pair is header name
		if (header != data.size() - 1) file01 << ","; // no comma at the end of line
	}
	file01 << "\n"; // to the next row
	// send datas to the stream
	for (int val = 0; val < data.at(0).second.size(); ++val) {
		for (int col = 0; col < data.size(); ++col) {
			file01 << data.at(col).second.at(val);
			if (col != data.size() - 1) file01 << ",";
		}
		file01 << "\n"; // to the next row
	}
	file01.close();
	return COMPLETE;
}
//bool import datafile(std::string dir_path); // default file type:　csv file
#endif
// the latest version: 2020/08/23 -B.R.Tseng
//------------------------------------------------------------------------
// 2020/08/23: replace std::vector<double> with arma::Col<double>