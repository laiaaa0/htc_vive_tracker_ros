#include "hand_eye_calibration_helper.h"
tf::Transform HandEyeHelper::GetBaseWorldFromTransforms ( 	tf::Transform hand_eye , tf::Transform base_hand, tf::Transform world_eye){

	
	tf::Transform  t1 = base_hand*hand_eye;
	t1 = t1*(world_eye.inverse());
	return t1;

}

tf::Transform HandEyeHelper::GetBaseFromFilePaths (const std::string & hand_eye_json_path, const std::string &  base_hand_csv, const std::string & world_eye_csv){
	
	FileReader fr;
	tf::Transform hand_eye_t = fr.ReadTransformFromJSON (hand_eye_json_path);
	tf::Transform base_hand_t = fr.ReadFirstTransformFromCSV (base_hand_csv);
	tf::Transform world_eye_t = fr.ReadFirstTransformFromCSV (world_eye_csv);
	
	return GetBaseWorldFromTransforms (hand_eye_t, base_hand_t, world_eye_t);
}
