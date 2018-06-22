#include "file_reader.h"
	
	
std::vector<std::string> string_values = {"x","y","z","i","j","k","w"};
tf2::Transform FileReader::ReadTransformFromJSON (const std::string & json_file_path){
	std::ifstream json_file(json_file_path);
	//std::cout<"Reading JSON FILE : "<<std::endl;
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

	
	tf2::Transform transform = GetTransformFromVector(json_values);
	return transform;
}


tf2::Transform FileReader::ReadFirstTransformFromCSV (const std::string & csv_file_path){
	std::ifstream csv_file(csv_file_path);
	//only get the first line
	std::vector<double> csv_values(8,0.0); //time, x,y,z, qx,qy,qz,qw
	tf2::Transform transform;
	std::string data_value;
	for (int i = 0; i < 8 ; ++i){
		if (csv_file.good()){
			getline(csv_file,data_value,',');
			csv_values[i] = std::stod(data_value);
			std::cout<<csv_values[i]<<std::endl;
		}
	}
	transform = (GetTransformFromVector (csv_values));
	//	transform_stamped.header.stamp_ = ros::Time  (csv_values[0]);
	return transform;	

}


tf2::Transform FileReader::GetTransformFromVector(const std::vector<double>&values){
	std::cout<<"size ' "<<values.size()<<std::endl;
	tf2::Transform transform;
	int first_value_position = 0;
	if (values.size()==7 || values.size()==8){
		if (values.size() == 8) first_value_position = 1;
		
		transform.setOrigin(
				tf2::Vector3(values[first_value_position],
						values[first_value_position+1],
						values[first_value_position+2]));
		transform.setRotation(tf2::Quaternion(
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
tf2::Transform FileReader::GetBaseWorldFromTransforms ( 	tf2::Transform hand_eye , tf2::Transform base_hand, tf2::Transform world_eye){
	
	tf2::Transform  t1 = base_hand*hand_eye;
	t1 = t1*(world_eye.inverse());
	return t1;

}

tf2::Transform FileReader::GetBaseFromFilePaths (const std::string & hand_eye_json_path, const std::string &  base_hand_csv, const std::string & world_eye_csv){
	
	tf2::Transform hand_eye_t = ReadTransformFromJSON (hand_eye_json_path);
	tf2::Transform base_hand_t = ReadFirstTransformFromCSV (base_hand_csv);
	tf2::Transform world_eye_t = ReadFirstTransformFromCSV (world_eye_csv);
	
	return GetBaseWorldFromTransforms (hand_eye_t, base_hand_t, world_eye_t);
}
