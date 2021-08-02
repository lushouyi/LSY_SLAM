# LSY_SLAM
# LSY_SLAM

**21 July 2021**:The front-end framework has been preliminarily built, and the feature point tracking has been implemented using the optical-flow method. The next step will be to try to optimize the front-end using the 2-point RANSAC and the pair-pole constraint approach.


**31 July 2021**:The front-end has been built. Based on the previous code, I added the the pair-pole constraint and the 2-point RANSAC algorithm to reject the outlier. But the two-point RANSAC algorithm did not work well, I guess because I used the raw IMU data (containing bias and noise). In the back-end we will correct the IMU measurements. The two-point RANSAC algorithm will be further optimized when the back-end is built.

LSY_SLAM is a real-time multi-sensor fusion SLAM system based on my graduation design. I mainly refer to VINS-Mono, MSCKF and DSO to complete the construction of the framework. This system will be improved all the time, and my PhD will continue the research of visual SLAM. What's learned from books is superficial after all. It's crucial to have it personally tested somehow. Improve myself in practice.

## 1. Prerequisites
1.1 **Ubuntu** and **ROS**
Ubuntu  18.04.
ROS Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)
additional ROS pacakge
```
    sudo apt-get install ros-YOUR_DISTRO-cv-bridge ros-YOUR_DISTRO-tf ros-YOUR_DISTRO-message-filters ros-YOUR_DISTRO-image-transport
```


1.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html), remember to **make install**.
(Our testing environment: Ubuntu 18.04, ROS Melodic, OpenCV 3.2.1, Eigen 3.3.3) 

## 2. Build LSY_SLAM on ROS
Clone the repository and catkin_make:
```
    git clone https://github.com/lushouyi/LSY_SLAM.git
    cd LSY_SLAM
    catkin_make
    source ~/devel/setup.bash
```

## 3. Run
Open three terminals, launch the front_end , rviz and play the bag file respectively. Take MH_01 for example
```
    roslaunch front_end euroc.launch 
    rviz
    rosbag play YOUR_PATH_TO_DATASET/MH_01_easy.bag 
```
