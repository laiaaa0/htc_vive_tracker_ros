// Copyright (C) 2010-2011 Institut de Robotica i Informatica Industrial, CSIC-UPC.
// Author 
// All rights reserved.
//
// This file is part of iri-ros-pkg
// iri-ros-pkg is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
// 
// IMPORTANT NOTE: This code has been generated through a script from the 
// iri_ros_scripts. Please do NOT delete any comments to guarantee the correctness
// of the scripts. ROS topics can be easly add by using those scripts. Please
// refer to the IRI wiki page for more information:
// http://wikiri.upc.es/index.php/Robotics_Lab

#ifndef _htc_vive_tracker_alg_node_h_
#define _htc_vive_tracker_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "htc_vive_tracker_alg.h"

// [publisher subscriber headers]
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include "hand_eye_calibration_helper.h"
// [service client headers]

// [action server client headers]

#include <std_srvs/Trigger.h>
/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class HtcViveTrackerAlgNode : public algorithm_base::IriBaseAlgorithm<HtcViveTrackerAlgorithm>
{
  private:
    // [publisher attributes]
    const std::string BASE_NAME = "iri_wam_link_base";
    const std::string WORLD_NAME = "chaperone";
    geometry_msgs::TransformStamped transform_stamped_;
    tf::StampedTransform transform_wam_chaperone_;
    float ax_, ay_,az_,angle_rad_;
    bool apply_rotation_;
    double wam_to_chaperone_x_, wam_to_chaperone_y_, wam_to_chaperone_z_;
    double wam_to_chaperone_i_, wam_to_chaperone_j_, wam_to_chaperone_k_, wam_to_chaperone_w_;
 

    HandEyeHelper hand_eye_helper_;
    // [subscriber attributes]
	
    // [service attributes]

    // [client attributes]

    // [action server attributes]

    // [action client attributes]

   /**
    * \brief config variable
    *
    * This variable has all the driver parameters defined in the cfg config file.
    * Is updated everytime function config_update() is called.
    */
    Config config_;
    std::string device_name_;
    uint32_t haptic_pulse_strength_;
    bool publish_hmd_;
 
    ros::ServiceServer trigger_pulse_server_;
    bool trigger_pulse_serverCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  public:
   /**
    * \brief Constructor
    * 
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    HtcViveTrackerAlgNode(void);

   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~HtcViveTrackerAlgNode(void);

  protected:
   /**
    * \brief main node thread
    *
    * This is the main thread node function. Code written here will be executed
    * in every node loop while the algorithm is on running state. Loop frequency 
    * can be tuned by modifying loop_rate attribute.
    *
    * Here data related to the process loop or to ROS topics (mainly data structs
    * related to the MSG and SRV files) must be updated. ROS publisher objects 
    * must publish their data in this process. ROS client servers may also
    * request data to the corresponding server topics.
    */
    void mainNodeThread(void);

   /**
    * \brief dynamic reconfigure server callback
    * 
    * This method is called whenever a new configuration is received through
    * the dynamic reconfigure. The derivated generic algorithm class must 
    * implement it.
    *
    * \param config an object with new configuration from all algorithm 
    *               parameters defined in the config file.
    * \param level  integer referring the level in which the configuration
    *               has been changed.
    */
    void node_config_update(Config &config, uint32_t level);

   /**
    * \brief node add diagnostics
    *
    * In this abstract function additional ROS diagnostics applied to the 
    * specific algorithms may be added.
    */
    void addNodeDiagnostics(void);

    // [diagnostic functions]
    
    // [test functions]
    void BroadcastPoseRotated(const std::string & device_name);
    void BroadcastWAMToChaperoneTransformation();
    void PrintQuaternionPose(const std::string & device_name);
    void PrintAllDeviceNames();
    void ApplyRotation(tf::Quaternion & q, float x, float y, float z, float angle);
    tf::Quaternion ApplyRotationForIRIStandardCoordinates(const tf::Quaternion & orig);
    void SetValuesWamToChaperone(const std::string & hand_eye_json_path, const std::string &  base_hand_csv, const std::string & world_eye_csv);

};

#endif
