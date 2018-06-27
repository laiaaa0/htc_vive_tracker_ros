### ROS node for htc vive
Publishes all the devices positions and orientations as a tf tree. The parent of the tree is called the chaperone. The coordinate system is the following

![tf_tree](/uploads/805c03196bbe7561d47c843e24231517/tf_tree.png)


# Hand eye calibration

Use the functions from [ethz-asl/hand_eye_calibration](https://github.com/ethz-asl/hand_eye_calibration)

This will generate a file calibration.json, which is similar to the one found [here](cfg/calibrationHandEye.json). This file contains HAND-EYE transformation.
The scripts also generate two aligned_...csv files, which are the tf poses aligning the time.

Take these three files and call the function in the class HandEyeHelper, as follows

    HandEyeHelper hand_eye_helper;   
    hand_eye_helper.GetBaseFromFilePaths("cfg/calibrationHandEye.json","cfg/aligned_tf_poses_wam.csv","cfg/aligned_tf_poses_tracker.csv");
    
This returns a transform, you can then print it and add it to your launch file

# Tracker axis

![tracker_axis](/uploads/248d343fc155216408d836e061636a7d/tracker_axis.png)

# Controller axis

![controller_axis](/uploads/9fb896ac8d6049efccea833d0562715c/controller_axis.png)

<!--# Class reference

![htcvivetrackerROS-Page-2](/uploads/b54f6fac94401873783ee96495b84694/htcvivetrackerROS-Page-2.jpg)

![htcvivetrackerROS](/uploads/011da2e5568b6ee11056e188056c98d3/htcvivetrackerROS.jpg)-->