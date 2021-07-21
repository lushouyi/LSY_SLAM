//
// Created by lsy on 2021/7/8.
//
#include "parameters.h"

int ROW;
int COL;
extern int FOCAL_LENGTH;
int TRACK_INTEN;
std::string IMAGE_TOPIC;
std::string IMU_TOPIC;
int MAX_CNT;
int MIN_DIST;
cv::Mat K;
std::vector<float> distortion_coeffs(4); //指定容器的个数

template<class T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    else
    {
        ROS_ERROR_STREAM("Failed to load: "<< name);
        n.shutdown();
    }
    return ans;
}

void readParameters(ros::NodeHandle &n)
{
    std::string config_file;
    config_file = readParam<std::string>(n, "config_file");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);

    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["image_topic"] >> IMAGE_TOPIC;
    fsSettings["imu_topic"] >> IMU_TOPIC;
    fsSettings["image_width"] >> COL;
    fsSettings["image_height"] >> ROW;
    fsSettings["max_cut"] >> MAX_CNT;
    fsSettings["min_dist"] >> MIN_DIST;
    fsSettings["track_intensity"] >> TRACK_INTEN;
    K = cv::Mat::eye(3,3, CV_32F);
    fsSettings["intrinsics"][0] >> K.at<float>(0,0);  //fx
    fsSettings["intrinsics"][2] >> K.at<float>(0,2);  //cx
    fsSettings["intrinsics"][1] >> K.at<float>(1,1);  //fy
    fsSettings["intrinsics"][3] >> K.at<float>(1,2);  //cy
    fsSettings["distorrion_parameters"][0] >> distortion_coeffs[0];
    fsSettings["distorrion_parameters"][1] >> distortion_coeffs[1];
    fsSettings["distorrion_parameters"][2] >> distortion_coeffs[2];
    fsSettings["distorrion_parameters"][3] >> distortion_coeffs[3];

    fsSettings.release();
}