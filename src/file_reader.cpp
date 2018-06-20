#include "file_reader.h"
	
	
tf2::Transform FileReader::ReadTransformFromJSON (const std::string & json_file_path){
	std::ifstream json_file(json_file_path);
	std::string line;
	while (json_file.good()){
		getline(json_file,line);
	}

	tf2::Transform transform;
	return transform;
}
tf2::Transform FileReader::ReadTransformFromXML (const std::string & xml_file_path){
	std::ifstream xml_file(xml_file_path);
	std::string line;
	while (xml_file.good()){
		getline(xml_file,line);
	}

	tf2::Transform transform;
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
tf2::Transform GetBaseWorldFromTransforms ( 	tf2::Transform hand_eye , tf2::Transform base_hand, tf2::Transform world_eye){
	
	tf2::Transform  t1 = base_hand*hand_eye;
	t1 = t1*(world_eye.inverse());
	return t1;

}
