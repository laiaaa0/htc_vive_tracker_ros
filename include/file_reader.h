#include <sstream> 
#include <string>
#include <iostream>
#include <fstream>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
class FileReader{
	
	public:
	double ValueFromString (const std::string & value_str);
	tf2::Transform GetTransformFromVector (const std::vector<double> & values);
	tf2::Transform ReadTransformFromJSON (const std::string & json_file_path);
	tf2::Transform ReadFirstTransformFromCSV (const std::string & csv_file_path);
	tf2::Transform GetBaseWorldFromTransforms ( 	tf2::Transform hand_eye , tf2::Transform base_hand, tf2::Transform world_eye);
	tf2::Transform GetBaseFromFilePaths (const std::string & hand_eye_json_path, const std::string &  base_hand_csv, const std::string & world_eye_csv);

};
