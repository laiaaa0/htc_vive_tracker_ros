#include "htc_vive_tracker_alg_node.h"

HtcViveTrackerAlgNode::HtcViveTrackerAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<HtcViveTrackerAlgorithm>()
{
   std::string base_path;
   if (this->public_node_handle_.hasParam("base_path")){
    this->public_node_handle_.getParam("base_path",base_path);
   } else {
    ROS_INFO ("Using default base path!");
    base_path = "/home/pmlab/iri-lab/iri_ws/src/iri_htc_vive_tracker/cfg/" ;
   }

   int loop_rate;
   if (this->public_node_handle_.hasParam("loop_rate")){
	this->public_node_handle_.getParam("loop_rate", loop_rate);
   } else {
	ROS_INFO ("Using default loop_rate!");
	loop_rate = 20;
   }

   this->loop_rate_ = loop_rate;
   bool verbose = true; 

   if (this->public_node_handle_.hasParam("verbose")){
	this->public_node_handle_.getParam("verbose", verbose);
   }


   std::string filename1 = "calibrationHandEye.json";
   std::string filename2 = "aligned_tf_poses_wam.csv";
   std::string filename3 = "aligned_tf_poses_tracker.csv";
   bool values_set_ok = this->SetValuesWamToChaperone(
	base_path+filename1,
	base_path+filename2,
	base_path+filename3);
 
  if (!values_set_ok) {
	ROS_ERROR("Problem with setting values!");
  }
  if (!this->alg_.InitVR(verbose)) {
  	ROS_ERROR("Problem with initialization. Check other error messages");
  }

  // [init publishers]
  
  
  // [init subscribers]
  
  // [init services]
  
  this->trigger_pulse_server_ = this->public_node_handle_.advertiseService("trigger_pulse", &HtcViveTrackerAlgNode::trigger_pulse_serverCallback, this);

  this->get_button_server_ = this->public_node_handle_.advertiseService("button_pressed", &HtcViveTrackerAlgNode::get_button_serverCallback,this);
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

HtcViveTrackerAlgNode::~HtcViveTrackerAlgNode(void)
{
  // [free dynamic memory]
}


void HtcViveTrackerAlgNode::mainNodeThread(void)
{

  // This function detects if a new device has been connected / disconnected
  this->alg_.PollEvents();

  // For each device name 
  std::vector<std::string> names;
  if (this->alg_.GetDeviceNames(names)) {
	for (int i = 0; i<names.size(); ++i) {
		if (this->alg_.GetBatteryLevel(names[i]) <= 0.1) {
			ROS_WARN_STREAM ("Battery level of "<<names[i]<<" is less than 10%");
		}
		
		// HMD always returns a dummy position which is either (0,0,0,1) or the position of a tracking_reference.
		if (names[i]=="hmd_1" && !this->publish_hmd_) {
			continue;
		}
		else{
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
 
  if (config.print_devices) {
	this->PrintAllDeviceNames();
  }
  this->haptic_pulse_strength_ = config.pulse_length;
 
  this->config_=config;
  this->publish_hmd_ = config.publish_hmd;
  this->alg_.unlock();
}

void HtcViveTrackerAlgNode::addNodeDiagnostics(void)
{
}

void HtcViveTrackerAlgNode::BroadcastPoseRotated(const std::string & device_name) {

    static tf::TransformBroadcaster tf_broadcaster;
    double pose[3];
    double quaternion[4];
    double roll,pitch,yaw;
	
    if (this->alg_.GetDevicePositionQuaternion(device_name,pose,quaternion)) {

	this->transform_stamped_.header.stamp = ros::Time::now();
	this->transform_stamped_.header.frame_id = WORLD_NAME;
	transform_stamped_.child_frame_id = device_name;
	transform_stamped_.transform.translation.x = pose[0];
	transform_stamped_.transform.translation.y = pose[1];
	transform_stamped_.transform.translation.z = pose[2];
	
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


void HtcViveTrackerAlgNode::PrintAllDeviceNames() {
	std::vector<std::string> names;
	if (this->alg_.GetDeviceNames(names)) {
		std::cout<<"Devices : "<<std::endl;
		for (int i=0; i<names.size(); ++i) {
			if (names[i]!="") std::cout<<"  * "<<names[i]<<std::endl;
		}
	}
}
tf::Quaternion HtcViveTrackerAlgNode::ApplyRotationForIRIStandardCoordinates(const tf::Quaternion & orig) {
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

void HtcViveTrackerAlgNode::BroadcastWAMToChaperoneTransformation() {
    	static tf::TransformBroadcaster tf_broadcaster;
	tf_broadcaster.sendTransform(this->transform_wam_chaperone_);
}


bool HtcViveTrackerAlgNode::SetValuesWamToChaperone(const std::string & hand_eye_json_path, const std::string &  base_hand_csv, const std::string & world_eye_csv) {
	tf::Transform base_transform = this->hand_eye_helper_.GetBaseFromFilePaths(hand_eye_json_path, base_hand_csv, world_eye_csv);
	std::cout<<"transform x "<<base_transform.getOrigin().x()<<std::endl;
	std::cout<<"transform y "<<base_transform.getOrigin().y()<<std::endl;
	std::cout<<"transform z "<<base_transform.getOrigin().z()<<std::endl;
	std::cout<<"transform i "<<base_transform.getRotation().x()<<std::endl;
	std::cout<<"transform j "<<base_transform.getRotation().y()<<std::endl;
	std::cout<<"transform k "<<base_transform.getRotation().z()<<std::endl;
	std::cout<<"transform w "<<base_transform.getRotation().w()<<std::endl;
	this->transform_wam_chaperone_ = tf::StampedTransform(
		 base_transform,
		 ros::Time::now(),
		 BASE_NAME,
		WORLD_NAME
	);
	if (std::isnan(base_transform.getOrigin().x())) return false;
	return true;
}


bool HtcViveTrackerAlgNode::trigger_pulse_serverCallback(iri_htc_vive_tracker::TriggerHapticPulse::Request &req, iri_htc_vive_tracker::TriggerHapticPulse::Response &res) {
	res.success = this->alg_.TriggerHapticPulse(req.device_name, this->haptic_pulse_strength_);
	if (!res.success) {
		res.message = DEVICE_NOT_FOUND_MSG;
	}

	return true;

}

bool HtcViveTrackerAlgNode::get_button_serverCallback(iri_htc_vive_tracker::GetButtonPressed::Request &req, iri_htc_vive_tracker::GetButtonPressed::Response &res) {
	vr::EVRButtonId button_pressed =  this->alg_.GetPressedButton(req.device_name);
	res.button_pressed = (int) button_pressed;
	res.success = this->alg_.IsDeviceDetected (req.device_name);
	if (!res.success) res.message = DEVICE_NOT_FOUND_MSG;
	return true;
}
/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<HtcViveTrackerAlgNode>(argc, argv, "htc_vive_tracker_alg_node");
}
