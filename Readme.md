### ROS node for htc vive
Publishes all the devices positions and orientations as a tf tree. The parent of the tree is called the chaperone. The coordinate system is the following

![tf_tree](/uploads/805c03196bbe7561d47c843e24231517/tf_tree.png)


# Hand eye calibration

Use the functions from [HandEyeMatlab](https://gitlab.iri.upc.edu/lfreixas/HandEyeCalibrationMatlab)


This returns the two transforms (base-world and hand-eye) , you can then print it and add it to your launch file

Check the launch file [example](launch/publish_wam_chaperone_link.launch)


# Tracker axis

![tracker_axis](/uploads/248d343fc155216408d836e061636a7d/tracker_axis.png)

# Controller axis

![controller_axis](/uploads/9fb896ac8d6049efccea833d0562715c/controller_axis.png)

<!--# Class reference

![htcvivetrackerROS-Page-2](/uploads/b54f6fac94401873783ee96495b84694/htcvivetrackerROS-Page-2.jpg)

![htcvivetrackerROS](/uploads/011da2e5568b6ee11056e188056c98d3/htcvivetrackerROS.jpg)-->


# WAM follows tracker

## Dependencies
* iri_wam_dmp_tracker
* iri_tf_to_pose

## Procedure

To test the WAM following tracker demo do the following :

    roslaunch iri_htc_vive_tracker wam_follow_tracker.launch
    roslaunch iri_wam_bringup iri_wam_bringup.launch
    roslaunch iri_wam_dmp_tracker iri_wam_dmp_tracker.launch