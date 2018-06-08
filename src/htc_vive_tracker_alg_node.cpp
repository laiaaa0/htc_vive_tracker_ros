#include "htc_vive_tracker_alg_node.h"

HtcViveTrackerAlgNode::HtcViveTrackerAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<HtcViveTrackerAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 5;//in [Hz]
  if (!this->alg_.InitVR(true)){
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
bool AreArraysEqual (const double * array1, const double * array2){
	//if (array1.size() != array2.size()) return false;
	int i = 0;
	while (array1[i]!=NULL){
		if (array1[i] != array2[i]) return false;
		i++;
	}
	return true;

}
bool HtcViveTrackerAlgNode::CheckHMDValuesValid (){
    double position[3];
    double quat [4];
    double position2[3];
    double quat2 [4];
    if (this->alg_.GetDevicePositionQuaternion ("hmd_1", position,quat)){
 	if (position[0]==0 && position[1] == 0 && position[2] == 0){
		if (quat[0]==0 && quat[1] == 0 && quat[2] == 0 && quat[3] == 1){
			return false;
		}
	}
	else {
		if (this->alg_.GetDevicePositionQuaternion ("tracking_reference_2", position2, quat2)){
			if (AreArraysEqual (position, position2)){
				return false;
			}
		}
		
	}
    }
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

 //PrintQuaternionPose("tracker_1");
 // BroadcastPose(this->device_name_);
  std::vector<std::string> names;
  if (this->alg_.GetDeviceNames(names)){
	for (int i = 0; i<names.size(); ++i){
		if (names[i]=="hmd_1" && !this->CheckHMDValuesValid()){
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
  this->alg_.unlock();
}

void HtcViveTrackerAlgNode::addNodeDiagnostics(void)
{
}

void HtcViveTrackerAlgNode::BroadcastPoseRotated(const std::string & device_name){

    static tf2_ros::TransformBroadcaster tf_broadcaster;
    double pose[3];
    double quaternion[4];
    double roll,pitch,yaw;
	
    if (this->alg_.GetDevicePositionQuaternion(device_name,pose,quaternion)){
    //if (this->alg_.GetDevicePositionEuler(device_name,pose,roll,pitch,yaw)){
	this->transform_stamped_.header.stamp = ros::Time::now();
	this->transform_stamped_.header.frame_id = "chaperone";
	transform_stamped_.child_frame_id = device_name;
	transform_stamped_.transform.translation.x = pose[0];
	transform_stamped_.transform.translation.y = pose[1];
	transform_stamped_.transform.translation.z = pose[2];

    	tf2::Quaternion q = tf2::Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
	
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
tf2::Quaternion HtcViveTrackerAlgNode::ApplyRotationForIRIStandardCoordinates(const tf2::Quaternion & orig){
	tf2::Quaternion q_result;

	//Define rotations by axis + angle
	tf2::Vector3 x_axis(1.0, 0.0, 0.0);	
	tf2::Vector3 z_axis(0.0, 0.0, 1.0);	
	tf2::Quaternion rotation_x90 = tf2::Quaternion(x_axis, M_PI/2);
	tf2::Quaternion rotation_x180 = tf2::Quaternion(x_axis, M_PI);
	tf2::Quaternion rotation_z90 = tf2::Quaternion(z_axis, M_PI/2);

	//Apply two pre-rotations, in order to match axis in device
	//Apply two post-rotations, in order to match axis to coordinate system.
	q_result = rotation_z90*rotation_x90*orig*rotation_x180*rotation_z90;
	
	return q_result;
	
}
void HtcViveTrackerAlgNode::ApplyRotation(tf2::Quaternion & orig, float ax, float ay, float az, float angle_radians){
	tf2::Vector3 axis(ax,ay,az);
	tf2::Quaternion rotation = tf2::Quaternion(axis, angle_radians);
	
	tf2::Quaternion finalQ = rotation*orig;
	
	this->transform_stamped_.transform.rotation.x = finalQ.x();
	this->transform_stamped_.transform.rotation.y = finalQ.y();
	this->transform_stamped_.transform.rotation.z = finalQ.z();
	this->transform_stamped_.transform.rotation.w = finalQ.w();
	
	
}
/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<HtcViveTrackerAlgNode>(argc, argv, "htc_vive_tracker_alg_node");
}
