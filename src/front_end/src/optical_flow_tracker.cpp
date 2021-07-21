//
// Created by lsy on 2021/6/30.
//

#include "optical_flow_tracker.h"
#include "parameters.h"
//const int col = 1920;
//const int row = 1080;
int OpticalFlowTracker::n_id = 0;

OpticalFlowTracker::OpticalFlowTracker()
{

}

bool outImage(const cv::Point2d &point)
{
    const int BORDER_SIZE = 1;
    int x = cvRound(point.x);
    int y = cvRound(point.y);
    return x <= BORDER_SIZE || x >= COL-BORDER_SIZE || y <= BORDER_SIZE || y >= ROW-BORDER_SIZE;
}

template<class T>
void reduceVector(vector<uchar> &status, vector<T> &points)
{
    int j = 0;
    for(int i=0; i<points.size(); i++)
        if(status[i] == 1)
            points[j++] = points[i];
    points.resize(j);
}

bool comp(const pair<int, pair<cv::Point2d, int>> &a, const pair<int, pair<cv::Point2d, int>> &b)
{
    return a.first > b.first;
}

void OpticalFlowTracker::setMask()
{
    mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;
    for(unsigned int i = 0; i < cur_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(cur_pts[i], ids[i])));
    sort(cnt_pts_id.begin(), cnt_pts_id.end(),comp);

    cur_pts.clear();
    track_cnt.clear();
    ids.clear();

    for(auto &it : cnt_pts_id)
    {
        if(mask.at<uchar>(it.second.first) == 255)
        {
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
            cur_pts.push_back(it.second.first);
            track_cnt.push_back(it.first);
            ids.push_back(it.second.second);
        }
    }
}

void OpticalFlowTracker::addPoints(int n_add_cnt)
{
    cv::goodFeaturesToTrack(cur_img, add_pts, n_add_cnt, 0.01, MIN_DIST, mask, 3, true, 0.04);
    for(auto &p : add_pts)
    {
        cur_pts.push_back(p);
        track_cnt.push_back(1);
        ids.push_back(n_id++);  //点的id从0开始
    }
}

void OpticalFlowTracker::readImage(const cv::Mat &_img, double _cur_time)
{
    cv::Mat img = _img;
    double cur_time = _cur_time;
    vector<uchar> status;
    vector<float> err;
    if(cur_img.empty())
        cur_img = img;
    else
    {
        prev_img = cur_img;
        cur_img = img;
        prev_pts = cur_pts;
        prev_un_pts = cur_un_pts;
        cur_un_pts.clear();
        cur_pts.clear();
        ROS_INFO_STREAM("prev_pts: " << prev_pts.size() <<" points");
        //cv::calcOpticalFlowPyrLK中点的坐标类型必须为单精度浮点数(float)
        cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21,21), 3);

        int count = 0;
        for(int i=0; i<int(cur_pts.size()); i++)
            if(status[i] && outImage(cur_pts[i]))
                status[i] = 0;

        ROS_INFO_STREAM("cur_pts_pre: " << cur_pts.size() <<" points");
        reduceVector<cv::Point2f>(status, cur_pts);
        reduceVector<int>(status, ids);
        reduceVector<int>(status, track_cnt);
        reduceVector<cv::Point2f>(status, cur_un_pts);

        for(auto &n : track_cnt)
            ++n;
        ROS_INFO_STREAM("cur_pts_flow: " << cur_pts.size() <<" points");
    }

    int n_add_cnt = MAX_CNT - static_cast<int>(cur_pts.size());
    if(n_add_cnt>20)
    {
        setMask();
        addPoints(n_add_cnt);
        ROS_INFO_STREAM("Added: " << n_add_cnt <<"points");
    }
    cv::Mat R = cv::Mat::eye(3,3, CV_32F);
    //如果不给参数R=K的话，去畸变函数给的是归一化坐标
    cv::undistortPoints(cur_pts, cur_un_pts, K, distortion_coeffs, R, K);
}