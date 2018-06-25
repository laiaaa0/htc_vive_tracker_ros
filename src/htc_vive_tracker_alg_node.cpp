#include "htc_vive_tracker_alg_node.h"

HtcViveTrackerAlgNode::HtcViveTrackerAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<HtcViveTrackerAlgorithm>()
{
   this->loop_rate_ = 20;
   bool verbose = true; 
   this-> SetValuesWamToChaperone(
     "/home/pmlab/iri-lab/iri_ws/src/iri_htc_vive_tracker/cfg/calibration.json",
    "/home/pmlab/iri-lab/iri_ws/src/iri_htc_vive_tracker/cfg/aligned_tf_poses_wam.csv",
    "/home/pmlab/iri-lab/iri_ws/src/iri_htc_vive_tracker/cfg/aligned_tf_poses_tracker.csv");


  // }

   //init class attributes if necessary
/*
  if (this->public_node_handle_.hasParam("looprate")){
    this->public_node_handle_.getParam("looprate",this->loop_rate_);
  }
  else {
	this->loop_rate_ = 5;
  }
  
  bool verbose = true;
  if (this->public_node_handle_.hasParam("verbose")){
    this->public_node_handle_.getParam("verbose",verbose);
  }
  
*/
  if (!this->alg_.InitVR(verbose)){
  	ROS_ERROR("Problem with initialization. Check other error messages");
  }
  // [init publishers]
  
  // [init subscribers]
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

HtcViveTrackerAlgNode::~HtcViveTrackerAlgNode(void)
{
  // [free dynamic memory]
}
bool AreArraysEqual(const double * array1, const double * array2){
	//if (array1.size() != array2.size()) return false;
	int i = 0;
	while(array1[i]!=NULL){
		if (array1[i] != array2[i]) return false;
		i++;
	}
	return true;

}
 
void HtcViveTrackerAlgNode::PrintQuaternionPose(const std::string & device){
    double pose[3];
    double q[4];
    if (this->alg_.GetDevicePositionQuaternion(device,pose,q)){
	for (int i = 0; i<3; ++i) std::cout<<pose[i]<<" ";
	std::cout<<std::endl;
	for (int i = 0; i<4; ++i) std::cout<<q[i]<<" ";
	std::cout<<std::endl;
    } 
}
void HtcViveTrackerAlgNode::mainNodeThread(void)
{

   BroadcastWAMToChaperoneTransformation ();
  // This function detects if a new device has been connected / disconnected
  this->alg_.PollEvents();

  // For each device name 
  std::vector<std::string> names;
  if (this->alg_.GetDeviceNames(names)){
	for (int i = 0; i<names.size(); ++i){
		if (this->alg_.GetBatteryLevel(names[i]) <= 0.1){
			ROS_WARN_STREAM ("Battery level of "<<names[i]<<" is less than 10%");
		}
		
		// HMD always returns a dummy position which is either (0,0,0,1) or the position of a tracking_reference.
		if (names[i]=="hmd_1" && !this->publish_hmd_){
			continue;
		}
		else{
  			ROS_INFO("Broadcasting pose of %s",names[i].c_str());
			BroadcastPoseRotated (names[i]);
		}
	}
  }
  
  // [fill msg structures]
  
  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]
}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void HtcViveTrackerAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->device_name_ = config.device_name;
  ROS_INFO("device name updated to %s",this->device_name_.c_str());
  if (config.print_devices){
	this->PrintAllDeviceNames();
  }/*
  if (config.apply_rotation){
	this->apply_rotation_ = true;
	this->ax_ = config.x_axis;
	this->ay_ = config.y_axis;
	this->az_ = config.z_axis;
	this->angle_rad_ = config.angle_degrees*M_PI/180;

  } else {
	this->apply_rotation_=false;
   }*/
  this->config_=config;
  this->publish_hmd_ = config.publish_hmd;
  this->alg_.unlock();
}

void HtcViveTrackerAlgNode::addNodeDiagnostics(void)
{
}

void HtcViveTrackerAlgNode::BroadcastPoseRotated(const std::string & device_name){

    static tf::TransformBroadcaster tf_broadcaster;
    double pose[3];
    double quaternion[4];
    double roll,pitch,yaw;
	
    if (this->alg_.GetDevicePositionQuaternion(device_name,pose,quaternion)){

	this->transform_stamped_.header.stamp = ros::Time::now();
	this->transform_stamped_.header.frame_id = "chaperone";
	transform_stamped_.child_frame_id = device_name;
	transform_stamped_.transform.translation.x = pose[0];
	transform_stamped_.transform.translation.y = pose[1];
	transform_stamped_.transform.translation.z = pose[2];
	
	/*

	x = quaternion[0]
 	y = quaternion[1]
	z = quaternion[2]
	
	w = quaternion[3]
	*/
    	tf::Quaternion q = tf::Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
	
	//Apply the necessary rotations so that the coordinate system is the one decided at IRI
	q = this -> ApplyRotationForIRIStandardCoordinates(q);

	transform_stamped_.transform.rotation.x = q.x();
	transform_stamped_.transform.rotation.y = q.y();
	transform_stamped_.transform.rotation.z = q.z();
	transform_stamped_.transform.rotation.w = q.w();


	tf_broadcaster.sendTransform(transform_stamped_);
    }

}
void HtcViveTrackerAlgNode::PrintAllDeviceNames(){
	std::vector<std::string> names;
	if (this->alg_.GetDeviceNames(names)){
		std::cout<<"Devices : "<<std::endl;
		for (int i=0; i<names.size(); ++i){
			if (names[i]!="") std::cout<<"  * "<<names[i]<<std::endl;
		}
	}
}
tf::Quaternion HtcViveTrackerAlgNode::ApplyRotationForIRIStandardCoordinates(const tf::Quaternion & orig){
	tf::Quaternion q_result;

	//Define rotations by axis + angle
	tf::Vector3 x_axis(1.0, 0.0, 0.0);	
	tf::Vector3 z_axis(0.0, 0.0, 1.0);	
	tf::Quaternion rotation_x90 = tf::Quaternion(x_axis, M_PI/2);
	tf::Quaternion rotation_x180 = tf::Quaternion(x_axis, M_PI);
	tf::Quaternion rotation_z90 = tf::Quaternion(z_axis, M_PI/2);

	//Apply two pre-rotations, in order to match axis in device
	//Apply two post-rotations, in order to match axis to coordinate system.
	q_result = rotation_z90*rotation_x90*orig*rotation_x180*rotation_z90;
	
	return q_result;
	
}
void HtcViveTrackerAlgNode::ApplyRotation(tf::Quaternion & orig, float ax, float ay, float az, float angle_radians){
	tf::Vector3 axis(ax,ay,az);
	tf::Quaternion rotation = tf::Quaternion(axis, angle_radians);
	
	tf::Quaternion finalQ = rotation*orig;
	
	this->transform_stamped_.transform.rotation.x = finalQ.x();
	this->transform_stamped_.transform.rotation.y = finalQ.y();
	this->transform_stamped_.transform.rotation.z = finalQ.z();
	this->transform_stamped_.transform.rotation.w = finalQ.w();
	
	
}
void HtcViveTrackerAlgNode::BroadcastWAMToChaperoneTransformation(){
	
    	static tf::TransformBroadcaster tf_broadcaster;

	tf_broadcaster.sendTransform(this->transform_wam_chaperone_);
 
}
void HtcViveTrackerAlgNode::SetValuesWamToChaperone(const std::string & hand_eye_json_path, const std::string &  base_hand_csv, const std::string & world_eye_csv){
	tf::Transform base_transform = this->hand_eye_helper_.GetBaseFromFilePaths(hand_eye_json_path, base_hand_csv, world_eye_csv);
	this->transform_wam_chaperone_ = tf::StampedTransform(
		 base_transform,
		 ros::Time::now(),
		 BASE_NAME,
		WORLD_NAME
	);
	
}
/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<HtcViveTrackerAlgNode>(argc, argv, "htc_vive_tracker_alg_node");
}
