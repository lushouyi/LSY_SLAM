//
// Created by lsy on 2021/7/8.
//

#ifndef LSY_SLAM_WS_PARAMETERS_H
#define LSY_SLAM_WS_PARAMETERS_H

#include<ros/ros.h>
#include <opencv2/highgui/highgui.hpp>

extern int ROW;
extern int COL;
extern int FOCAL_LENGTH;

extern std::string IMAGE_TOPIC;
extern std::string IMU_TOPIC;
extern int MAX_CNT;
extern int MIN_DIST;
extern cv::Mat K;
extern std::vector<float> distortion_coeffs;
extern int TRACK_INTEN;
extern int F_THRESHOLD;
extern double RANSAC_THRESHOLD;
extern cv::Mat extrinsicRotation;
void readParameters(ros::NodeHandle &n);



#endif //LSY_SLAM_WS_PARAMETERS_H
