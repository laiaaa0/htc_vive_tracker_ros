#include <sstream> 
#include <tf/transform_broadcaster.h>
#include <string>
#include <iostream>
#include <fstream>
#include <geometry_msgs/TransformStamped.h>
class FileReader{
	
	public:
	double ValueFromString (const std::string & value_str);
	tf::Transform GetTransformFromVector (const std::vector<double> & values);
	tf::Transform ReadTransformFromJSON (const std::string & json_file_path);
	tf::Transform ReadFirstTransformFromCSV (const std::string & csv_file_path);
};
