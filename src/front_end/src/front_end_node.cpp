//
// Created by lsy on 2021/6/29.
//
#include<ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <cv_bridge/cv_bridge.h>
#include "optical_flow_tracker.h"
#include "parameters.h"

ros::Publisher pub_image;
OpticalFlowTracker optical_flow_data;
void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
//********************ros message -> cv::Mat*********************
    cv_bridge::CvImageConstPtr ptr;
    ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    cv::Mat raw_img = ptr->image;

    optical_flow_data.readImage(raw_img, img_msg->header.stamp.toSec());

//**********************定义发送给后端的消息类型*************************
    sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
    sensor_msgs::ChannelFloat32 id_of_point;
    sensor_msgs::ChannelFloat32 u_of_point;
    sensor_msgs::ChannelFloat32 v_of_point;

    feature_points->header = img_msg->header;
    feature_points->header.frame_id = "feature_points";

    //对消息进行赋值
    auto &un_pts = optical_flow_data.cur_un_pts;
    auto &cur_pts = optical_flow_data.cur_pts;
    auto &ids = optical_flow_data.ids;

    for(unsigned int i=0; i<ids.size(); i++)
    {
        if(optical_flow_data.track_cnt[i] > 1)
        {
            geometry_msgs::Point32 p;  //定义点的消息类型
            p.x = un_pts[i].x;
            p.y = un_pts[i].y;
            p.z = 1;

            feature_points->points.push_back(p);
            id_of_point.values.push_back(ids[i]);
            u_of_point.values.push_back(un_pts[i].x);
            v_of_point.values.push_back(un_pts[i].y);
        }
    }

    feature_points->channels.push_back(id_of_point);
    feature_points->channels.push_back(u_of_point);
    feature_points->channels.push_back(v_of_point);

//******************往图像上画提取的特征点***************************
    ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8);
    cv::Mat show_img = ptr->image;
    cv::cvtColor(raw_img, show_img, CV_GRAY2RGB);
    for(unsigned int i=0; i<optical_flow_data.track_cnt.size(); i++)
    {
        float track_intensity = std::min(1.0, 1.0 * optical_flow_data.track_cnt[i] / TRACK_INTEN);
        cv::circle(show_img, optical_flow_data.cur_pts[i], 2, cv::Scalar(255 * (1 - track_intensity), 0, 255 * track_intensity), 2);
    }
    cv::imshow("vis", show_img);
    cv::waitKey(5);
    pub_image.publish(ptr->toImageMsg());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "front_end");  //frond_end is node's name
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(n);

    ros::Subscriber sub_img = n.subscribe(IMAGE_TOPIC, 100, img_callback);
    pub_image = n.advertise<sensor_msgs::Image>("usb_cam_raw_image", 1000);

    ros::spin();
    return 0;
}
