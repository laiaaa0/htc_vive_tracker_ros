#include "htc_vive_tracker_alg_node.h"

HtcViveTrackerAlgNode::HtcViveTrackerAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<HtcViveTrackerAlgorithm>()
{
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

   if (!this->alg_.InitVR(verbose)) {
   	ROS_ERROR("Problem with initialization. Check other error messages");
   }

    this->frame_names_set = true;
    if (this->public_node_handle_.hasParam("target_frame_name")){
        this->public_node_handle_.getParam("target_frame_name",this->target_frame_name_);
    }
    else {
        ROS_ERROR("No param set for target_frame");
        this->frame_names_set = false;
    }
    
    this->source_frame_name_ = "chaperone";

    pose_publisher_ = this->public_node_handle_.advertise<geometry_msgs::PoseStamped>("new_pose",100);
    vo_publisher_ = this->public_node_handle_.advertise<nav_msgs::Odometry>("vo",100);


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


void HtcViveTrackerAlgNode::BroadcastAllPoses(void) {
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


}

void HtcViveTrackerAlgNode::PublishPoseOfDeviceToFollow(void){
   if (this->frame_names_set){
    tf::StampedTransform stamped_transform;
    try{
        bool is_transform_possible = this->tf_listener_.canTransform(this->source_frame_name_, this->target_frame_name_, ros::Time(0));
        if (is_transform_possible) {
            this->tf_listener_.lookupTransform(this->source_frame_name_, this->target_frame_name_, ros::Time(0), stamped_transform);
            geometry_msgs::PoseStamped tf_pose = this->alg_.PoseFromTF(stamped_transform);
            Velocity device_vel = this->alg_.GetDeviceVelocity(this->target_frame_name_);
            nav_msgs::Odometry current_vo = this->CreateOdometryFromPoseVel(tf_pose, device_vel);
            this->vo_publisher_.publish(current_vo);
            this->pose_publisher_.publish(tf_pose);
        }
        else {
            ROS_INFO ("Transform not possible");
        }
    }
    catch (tf::TransformException &ex) {
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
    }
   }
  
}

void HtcViveTrackerAlgNode::mainNodeThread(void)
{

  // This function detects if a new device has been connected / disconnected
  this->alg_.PollEvents();

  // This function broadcasts the poses of all devices detected
  this->BroadcastAllPoses();

  // This function publishes the pose of the device in source_frame_name_
  this->PublishPoseOfDeviceToFollow();
  

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
	this->transform_stamped_.header.frame_id = this->alg_.WORLD_NAME;
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





bool HtcViveTrackerAlgNode::trigger_pulse_serverCallback(iri_htc_vive_tracker::TriggerHapticPulse::Request &req, iri_htc_vive_tracker::TriggerHapticPulse::Response &res) {
	res.success = this->alg_.TriggerHapticPulse(req.device_name, this->haptic_pulse_strength_);
	if (!res.success) {
		res.message = this->alg_.DEVICE_NOT_FOUND_MSG;
	}

	return true;

}

bool HtcViveTrackerAlgNode::get_button_serverCallback(iri_htc_vive_tracker::GetButtonPressed::Request &req, iri_htc_vive_tracker::GetButtonPressed::Response &res) {
	vr::EVRButtonId button_pressed =  this->alg_.GetPressedButton(req.device_name);
	res.button_pressed = (int) button_pressed;
	res.success = this->alg_.IsDeviceDetected (req.device_name);
	if (!res.success) res.message = this->alg_.DEVICE_NOT_FOUND_MSG;
	return true;
}
nav_msgs::Odometry HtcViveTrackerAlgNode::CreateOdometryFromPoseVel(const geometry_msgs::PoseStamped & pose, const Velocity & vel){
    
    geometry_msgs::TwistWithCovariance twist_msg;
    twist_msg.twist.linear.x = vel.linear_velocity.x;
    twist_msg.twist.linear.y = vel.linear_velocity.y;
    twist_msg.twist.linear.z = vel.linear_velocity.z;
    twist_msg.twist.angular.x = vel.angular_velocity.x;
    twist_msg.twist.angular.y = vel.angular_velocity.y;
    twist_msg.twist.angular.z = vel.angular_velocity.z;
    //TODO Fill covariance
    geometry_msgs::PoseWithCovariance pose_msg;
    pose_msg.pose = pose.pose;
    //TODO Fill covariance

    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = this->alg_.WORLD_NAME; 
    odom.child_frame_id = this->target_frame_name_;
    odom.pose = pose_msg;
    odom.twist = twist_msg;
    return odom;
}
/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<HtcViveTrackerAlgNode>(argc, argv, "htc_vive_tracker_alg_node");
}
