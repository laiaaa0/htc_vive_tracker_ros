#include "file_reader.h"
	
	
std::vector<std::string> string_values = {"x","y","z","i","j","k","w"};
tf::Transform FileReader::ReadTransformFromJSON (const std::string & json_file_path){
	std::ifstream json_file(json_file_path);
	std::string line;
	std::string data_value;
	std::vector<double>json_values(string_values.size(),0.0); // x,y,z, qx,qy,qz,qw

	while (json_file.good()){
		getline(json_file,line,'"');
		for (int i = 0; i<string_values.size(); ++i){
			if (line == string_values[i]){
				getline(json_file,data_value,',');
				json_values[i] = std::stod(data_value.erase(0,1));
			}
		}
	}

	
	tf::Transform transform = GetTransformFromVector(json_values);
	return transform;
}


tf::Transform FileReader::ReadFirstTransformFromCSV (const std::string & csv_file_path){
	std::ifstream csv_file(csv_file_path);
	//only get the first line
	std::vector<double> csv_values(8,0.0); //time, x,y,z, qx,qy,qz,qw
	tf::Transform transform;
	std::string data_value;
	for (int i = 0; i < 8 ; ++i){
		if (csv_file.good()){
			getline(csv_file,data_value,',');
			csv_values[i] = std::stod(data_value);
		}
	}
	transform = (GetTransformFromVector (csv_values));
	//	transform_stamped.header.stamp_ = ros::Time  (csv_values[0]);
	return transform;	

}


tf::Transform FileReader::GetTransformFromVector(const std::vector<double>&values){
	tf::Transform transform;
	int first_value_position = 0;
	if (values.size()==7 || values.size()==8){
		if (values.size() == 8) first_value_position = 1;
		
		transform.setOrigin(
				tf::Vector3(values[first_value_position],
						values[first_value_position+1],
						values[first_value_position+2]));
		transform.setRotation(tf::Quaternion(
					values[first_value_position+3],
					values[first_value_position+4],
					values[first_value_position+5],
					values[first_value_position+6]
					));
		
	}
	return transform;
	
	
}
double ValueFromString (const std::string & value_str){
	return std::stod (value_str);

}

