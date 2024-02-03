# MoCArU

MoCArU is a novel Motion Capture system based on ArUco developed by engineers at Tohoku University. The implementation given in this repository runs on ROS Noetic. In this tutorial, it is assumed that ROS Noetic is already installed. The following is a list of basic procedures necessary to operate MoCArU with image information from up to 10 cameras. By using lightweight camera stands, wireless communication, and a combination of odometry and ArUco recognition, MoCArU is specifically tailored for tracking swarm ground robot systems. It is a robust and cost-effective alternative to existing localization methods.

### Links

- [Laboratory news](https://www.rm.is.tohoku.ac.jp/mocaru) - MoCArU, Human-Robot Informatics Laboratory of Tohoku University

- [Paper](https://ieeexplore.ieee.org/document/10394661) - Published in 2023 IEEE Conference on Systems, Man, and Cybernetics (IEEE SMC) @Honolulu, HI, United States

### Before you begin

ArUco markers must be placed on the objects you wish to track (the default number of objects is 2). If you don't have an idea what ArUco is, it is recommended to read [this article](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html). Moreover, you can generate ArUco markers for printing out at [this website](https://chev.me/arucogen/). Afterward, it is left to you to install cameras in your physical environment and have images streamed constantly using MJPEG.


### Prerequisites

- Ubuntu 20.04 (Consider [Docker](https://www.docker.com/) if you don't have one)
- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)

These are other packages that are included as git-submodules: [(1)](https://github.com/yoshito-n-students/image_source) [(2)](https://github.com/yoshito-n-students/mjpeg_client) [(3)](https://github.com/yoshito-n-students/object_detection_msgs), and there are special instructions to modify certain codes in the packages prior to compiling them. If the `Prerequisites` are met already, please follow the instructions below, starting with cloning this repository at your preferred place. 

```bash
git clone --recursive https://github.com/BRN-Hub/MoCArU.git
```

To install all dependencies, we recommend using rosdep by running the following command. 

```bash
cd MoCArU/ros
rosdep install --from-paths src --ignore-src -r -y
```

It is recommended to set all files' permission in `src` folder to be executable. For example, you can use `sudo chmod -R 777 src` to grant full permission, but it is advised to check the security level as preferred in your environment. 

There are some modifications needed - e.g., changing some commands and adding more files. In this tutorial, I will summarize the case of using Sony Spresense boards equipped with HDR cameras as the image source. 

 - Move files from `requiredfiles/imagesourcelaunch` to `ros/src/image_source/launch/`. Note that if you use other cameras, you are most likely needed to recalibrate the cameras to obtain a `.yml` file, using [this calibration tool](https://docs.opencv.org/4.x/da/d13/tutorial_aruco_calibration.html). 
 - Move files from `requiredfiles/mjpeglaunch` to `ros/src/mjpeg_client/launch/`. Currently, `multicam_pic.launch` is set to obtain images from 10 cameras from defined IP addresses, which you are totally free to modify. 
 - Move the file from `requiredfiles/mjpegrviz` to `ros/src/mjpeg_client/rviz/`.

All the mentioned file-moving steps can be accomplished by following these commands. 

```bash
cd MoCArU
mv requiredfiles/imagesourcelaunch/* ros/src/image_source/launch/
mv requiredfiles/mjpeglaunch/* ros/src/mjpeg_client/launch/
mkdir -p ros/src/mjpeg_client/rviz && mv requiredfiles/mjpegrviz/* ros/src/mjpeg_client/rviz/
```

Source and compile. 

```bash
cd ros
catkin_make
source devel/setup.bash
```

**Steps to Start MoCArU: Receive Information from Camera**
Initiate the system to capture images sent from cameras over HTTP.

```bash
roslaunch mjpeg_client multicam_pic.launch
```

**Run ArUco Recognition**
To recognize specific ArUco markers, execute:

```bash
roslaunch aruco_pose_estimation marker_estimation.launch
```

If you wish to modify the ArUco ID or the size of ArUco marker (in the unit of m), adjust it in the launch file above.

**Run Camera Info**
This step is straightforward. Simply run:

```bash
roslaunch image_source camera_setting_info.launch
```

**Calculate Robot Position Relative to ArUco ID0**
You have two options to choose to proceed:
- To collect both average and Gaussian data combined from each camera:
```bash
roslaunch dist_from_tf info_combine_conv.launch
```

- To utilize Kalman data filtering (MoCArU), please use this launch file `info_combine_kalman.launch` instead. Either way, if your screen appears similar to the animation below, congratulations! 

**Outcome**
At this step, you will be able to check the position of your tracked objects as TF. We recommend visualizing it via RViz

![ill](../images/images/rvizex.gif?raw=true)

**(Optional) Try out our code with our Rosbag data**
We provide you with a bag file that contains sample image streaming, pose estimation, and velocity command data that we recorded during our test experiment. The bag file is `bag/2rob_exp.bag`, and it can be run with these bash scripts. We recommend using tmux to run each script below in a separate window, and you can observe the localization result via RViz. 

```bash
roscore
```


```bash
roslaunch aruco_pose_estimation marker_estimation.launch
```


```bash
roslaunch dist_from_tf info_combine_kalman.launch
```

```bash
rosbag play MoCArU/data/2rob_exp.bag
```

We recommend to enable only frames `turtle_kalman_t1` and `turtle_kalman_t2` while disabling others. 
