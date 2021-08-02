//
// Created by lsy on 2021/6/30.
//

#ifndef LSY_SLAM_WS_OPTICAL_FLOW_TRACKER_H
#define LSY_SLAM_WS_OPTICAL_FLOW_TRACKER_H

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <queue>
#include <sensor_msgs/Imu.h>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>

using namespace std;
using namespace Eigen;

class OpticalFlowTracker
{
public:
    OpticalFlowTracker();

    void readImage(const cv::Mat &_img, double _cur_time);
    void getImu();
    void setMask();
    void addPoints(int n_add_cnt);
    void rejectByF();
    void rejectByTwoPointRansac();

    cv::Mat prev_img, cur_img, forw_img;
    vector<cv::Point2f> prev_pts, cur_pts, forw_pts;
    vector<cv::Point2f> cur_un_pts, prev_un_pts;
    vector<cv::Point2f> add_pts;
    cv::Mat mask;
    vector<int> ids;
    vector<int> track_cnt;
    double cur_time = 0, prev_time = 0;
    static int n_id;
    vector<sensor_msgs::ImuConstPtr> imu_buf;


    double gyro_accum_x = 0, gyro_accum_y = 0, gyro_accum_z = 0;
    cv::Mat dR = cv::Mat::eye(3,3,CV_32F);

};

#endif //LSY_SLAM_WS_OPTICAL_FLOW_TRACKER_H