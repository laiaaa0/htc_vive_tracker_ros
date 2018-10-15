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
    



  // [init publishers]
  
    vo_publisher_ = this->public_node_handle_.advertise<nav_msgs::Odometry>("vo",100);
    pose_publisher_ = this->public_node_handle_.advertise<geometry_msgs::PoseStamped>("new_pose",100);
  
  // [init subscribers]
  
    filtered_odometry_subscriber_ = this->public_node_handle_.subscribe("filtered_odometry", 100, &HtcViveTrackerAlgNode::filtered_odometryCallback, this);
  
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
		if (names[i]=="hmd_1" && !this->config_.publish_hmd) {
			continue;
		}
		else{
			BroadcastPoseRotated (names[i]);
		}
	}
  }
}

void HtcViveTrackerAlgNode::mainNodeThread(void)
{

  // This function detects if a new device has been connected / disconnected
  this->alg_.PollEvents();

  // This function broadcasts the poses of all devices detected
  this->BroadcastAllPoses();

}

/*  [subscriber callbacks] */

void HtcViveTrackerAlgNode::filtered_odometryCallback(const nav_msgs::Odometry::ConstPtr & msg){
    geometry_msgs::PoseStamped new_pose;
    new_pose.pose = msg->pose.pose;
    new_pose.header.stamp = ros::Time::now();
    new_pose.header.frame_id = this->alg_.WORLD_NAME;
    pose_publisher_.publish(new_pose);
}

/*  [service callbacks] */

bool HtcViveTrackerAlgNode::trigger_pulse_serverCallback(iri_htc_vive_tracker::TriggerHapticPulse::Request &req, iri_htc_vive_tracker::TriggerHapticPulse::Response &res) {
	res.success = this->alg_.TriggerHapticPulse(req.device_name, this->config_.pulse_length);
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



/*  [action callbacks] */

/*  [action requests] */

void HtcViveTrackerAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
 
  if (config.print_devices) {
	  config.print_devices = false;
      this->PrintAllDeviceNames();
  }
 
  this->config_=config;
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


   	tf::Quaternion q = tf::Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
	
	//Apply the necessary rotations so that the coordinate system is the one decided at IRI
	q = this -> ApplyRotationForIRIStandardCoordinates(q);

	this->transform_stamped_.header.stamp = ros::Time::now();
	this->transform_stamped_.header.frame_id = this->alg_.WORLD_NAME;
    transform_stamped_.child_frame_id = device_name;
	transform_stamped_.transform.translation.x = pose[0];
	transform_stamped_.transform.translation.y = pose[1];
	transform_stamped_.transform.translation.z = pose[2];
	
	transform_stamped_.transform.rotation.x = q.x();
	transform_stamped_.transform.rotation.y = q.y();
	transform_stamped_.transform.rotation.z = q.z();
	transform_stamped_.transform.rotation.w = q.w();


	tf_broadcaster.sendTransform(transform_stamped_);
    //If the device is the one we want to follow, also publish the Odometry
    if (this->frame_names_set && device_name == this->target_frame_name_){
        geometry_msgs::Pose device_pose;
        device_pose.position.x = pose[0];
        device_pose.position.y = pose[1];
        device_pose.position.z = pose[2];
        device_pose.orientation.x = q.x();
        device_pose.orientation.y = q.y();
        device_pose.orientation.z = q.z();
        device_pose.orientation.w = q.w();
        Velocity device_vel = this->alg_.GetDeviceVelocity(this->target_frame_name_);
        nav_msgs::Odometry current_vo = this->CreateOdometryFromPoseVel(device_pose, device_vel);
        this->vo_publisher_.publish(current_vo);
    }
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
	tf::Vector3 y_axis(0.0, 1.0, 0.0);	
	tf::Vector3 z_axis(0.0, 0.0, 1.0);	
	tf::Quaternion rotation_x90 = tf::Quaternion(x_axis, M_PI/2);
	tf::Quaternion rotation_minusx90 = tf::Quaternion(x_axis, -M_PI/2);
	tf::Quaternion rotation_x180 = tf::Quaternion(x_axis, M_PI);
	tf::Quaternion rotation_z90 = tf::Quaternion(z_axis, M_PI/2);
	tf::Quaternion rotation_minusy90 = tf::Quaternion(y_axis, -M_PI/2);
	tf::Quaternion rotation_minusz90 = tf::Quaternion(z_axis, -M_PI/2);

	//Apply two pre-rotations, in order to match axis in device
	//Apply two post-rotations, in order to match axis to coordinate system.
	//q_result = rotation_z90*rotation_x90*orig*rotation_x180*rotation_z90;
	q_result = rotation_z90*rotation_x90*orig*rotation_minusy90*rotation_minusx90; 	
	return q_result;
	
}





nav_msgs::Odometry HtcViveTrackerAlgNode::CreateOdometryFromPoseVel(const geometry_msgs::Pose & pose, const Velocity & vel){
    
    geometry_msgs::TwistWithCovariance twist_msg;
    twist_msg.twist.linear.x = vel.linear_velocity.x;
    twist_msg.twist.linear.y = vel.linear_velocity.y;
    twist_msg.twist.linear.z = vel.linear_velocity.z;
    twist_msg.twist.angular.x = vel.angular_velocity.x;
    twist_msg.twist.angular.y = vel.angular_velocity.y;
    twist_msg.twist.angular.z = vel.angular_velocity.z;

    twist_msg.covariance = {0.01, 0, 0, 0, 0, 0,  // covariance on pose x
                                0, 0.01, 0, 0, 0, 0,  // covariance on pose y
                                0, 0, 0.01, 0, 0, 0,  // covariance on pose z
                                0, 0, 0, 1, 0, 0,  //  covariance on rot x
                                0, 0, 0, 0, 1, 0,  //  covariance on rot y
                                0, 0, 0, 0, 0, 1};  //  covariance on rot z

    geometry_msgs::PoseWithCovariance pose_msg;
    pose_msg.pose = pose;
    pose_msg.covariance = {0.01, 0, 0, 0, 0, 0,  // covariance on pose x
                                0, 0.01, 0, 0, 0, 0,  // covariance on pose y
                                0, 0, 0.01, 0, 0, 0,  // covariance on pose z
                                0, 0, 0, 1, 0, 0,  // covariance on rot x
                                0, 0, 0, 0, 1, 0,  // covariance on rot y
                                0, 0, 0, 0, 0, 1};  // covariance on rot z

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
