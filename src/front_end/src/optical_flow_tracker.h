//
// Created by lsy on 2021/6/30.
//

#ifndef LSY_SLAM_WS_OPTICAL_FLOW_TRACKER_H
#define LSY_SLAM_WS_OPTICAL_FLOW_TRACKER_H

#include <opencv2/opencv.hpp>

using namespace std;

class OpticalFlowTracker
{
public:
    OpticalFlowTracker();

    void readImage(const cv::Mat &_img, double _cur_time);
    void setMask();
    void addPoints(int n_add_cnt);

    cv::Mat prev_img, cur_img, forw_img;
    vector<cv::Point2f> prev_pts, cur_pts, forw_pts;
    vector<cv::Point2f> cur_un_pts, prev_un_pts;
    vector<cv::Point2f> add_pts;
    cv::Mat mask;
    vector<int> ids;
    vector<int> track_cnt;

    static int n_id;

};

#endif //LSY_SLAM_WS_OPTICAL_FLOW_TRACKER_H