#include <fstream>      // std::ifstream
#include <iostream>
#include <string>     // std::string, std::stod

struct Translation{
	long double x;
	double y;
	double z;
};

struct Orientation{
	double i;
	double j;
	double k;
	double w;
};
struct CalibrationInfo{
	double delay;
	Translation t;
	Orientation o;
};

bool IsNumber (char c){
	if (c>='0' && c<='9') return true;
	else if (c=='-') return true;
	else if (c=='.') return true;
	
	return false;
}
void AddValueToField (std::string & num, std::string & field, CalibrationInfo c){
	std::cout<<"adding "<<num<<" to "<< field<<std::endl;
	if (field == "x"){
		c.t.x = std::stod(num);
		std::cout<<c.t.x<<std::endl;
	} else if (field == "y"){
		c.t.y = std::stod(num);
	} else if (field == "z"){
		c.t.z = std::stod(num);
	}

	num = "";
	field = "";

}

CalibrationInfo ReadCalibrationFromJSONFile (const char * file_path){
	std::ifstream in_file_stream (file_path, std::ifstream::in);
	std::string current_field;
	std::string current_number;
	CalibrationInfo calibration_info;
	char c;
	bool is_open_quotes = false;
	while (in_file_stream.good()){
		c = in_file_stream.get();
		if (c == '"'){
			is_open_quotes = !is_open_quotes;
			
		} else if (IsNumber(c)){
			current_number = current_number+c;

		} else if (c == ',' || c == '{' || c == '}'){
			AddValueToField (current_number, current_field, calibration_info);
		}
		if (is_open_quotes && c!='"'){
			current_field = current_field + c;
	
		}
	//	std::cout<<c<<std::endl;
	}
	std::cout<<calibration_info.t.x<<std::endl;
}

int main (){
	CalibrationInfo ci = ReadCalibrationFromJSONFile("calibration.json");


}
