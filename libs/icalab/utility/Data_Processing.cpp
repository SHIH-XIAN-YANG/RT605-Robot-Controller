#include "Data_Processing.h"

bool export_datafile(std::string filename, datafile data){
	// -filename
	// -dataset: pair<(string)header name, (double)data value>
	//===============================================================================
	// Each column of data is represented by the pair <column name, column data>.
	//===============================================================================
	// create and output filestream object
	std::ofstream file01(filename);
	// build a dataframe
	// send the data header names to the stream(dataframe)
	for(int header = 0; header < data.size(); ++header){
		file01 << data.at(header).first; // the first element of the dataset pair is header name
		if(header != data.size() - 1) file01 << ","; // no comma at the end of line
	}
	file01 << "\n"; // to the next row
	// send datas to the stream
	for(int val = 0; val < data.at(0).second.size(); ++val){
		for(int col = 0; col < data.size(); ++col){
			file01 << data.at(col).second.at(val);
			if(col != data.size() - 1) file01 << ",";
		}
		file01 << "\n"; // to the next row
	}
	file01.close();
	return COMPLETE;
}
// the latest version: 2020/08/23 -B.R.Tseng
//------------------------------------------------------------------------
// 2020/08/23: replace std::vector<double> with arma::Col<double>