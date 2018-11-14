### ROS node for htc vive
Publishes all the devices positions and orientations as a tf tree. The parent of the tree is called the chaperone. The coordinate system is the following

![tf_tree](https://raw.githubusercontent.com/laiaaa0/htc_vive_tracker_ros/master/tf_tree.png)


# Hand eye calibration

Use the functions from [HandEyeMatlab](https://gitlab.iri.upc.edu/lfreixas/HandEyeCalibrationMatlab)


This returns the two transforms (base-world and hand-eye) , you can then print it and add it to your launch file

Check the launch file [example](launch/publish_wam_chaperone_link.launch)


# WAM follows tracker

## Dependencies
#### HTC Vive Tracker C++ library
Folllow the download and install instructions from the [htc_vive_tracker](https://gitlab.iri.upc.edu/labrobotica/drivers/htc_vive_tracker) repository.
#### IRI wam dmp tracker
Folllow the download and install instructions from the [iri_wam_dmp_tracker](https://gitlab.iri.upc.edu/iri_wam/iri_wam_dmp_tracker) repository.
#### Robot localization package

    sudo apt-get install ros-kinetic-robot-localization



## Procedure

To test the WAM following a tracker demo do the following :

    roslaunch iri_htc_vive_tracker wam_follow_device.launch  device:=tracker_1
    roslaunch iri_wam_bringup iri_wam_bringup.launch
    roslaunch iri_wam_dmp_tracker iri_wam_dmp_tracker.launch


# Coordinate systems
### Tracker axis

<img src="https://raw.githubusercontent.com/laiaaa0/htc_vive_tracker_ros/master/tracker_axis.png"  width="500" >


### Controller axis

<img src="https://raw.githubusercontent.com/laiaaa0/htc_vive_tracker_ros/master/Controller_axis.png" height="300">



