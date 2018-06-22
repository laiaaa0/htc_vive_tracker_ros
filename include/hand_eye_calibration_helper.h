
#include "file_reader.h"
class HandEyeHelper{
    public:
	tf::Transform GetBaseWorldFromTransforms ( 	tf::Transform hand_eye , tf::Transform base_hand, tf::Transform world_eye);
	tf::Transform GetBaseFromFilePaths (const std::string & hand_eye_json_path, const std::string &  base_hand_csv, const std::string & world_eye_csv);



};
