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

template<class T>
void reduceArray(Array<bool, 1, Dynamic> &status, vector<T> &points)
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

void OpticalFlowTracker::getImu()
{

    if(imu_buf.empty() || prev_time == 0)
        return;
    if(!(imu_buf.back()->header.stamp.toSec() >= cur_time))
        return;
    if(!(imu_buf.front()->header.stamp.toSec() <= prev_time))
        return;

    int n_omega_reading = 0;
    double omega_accum_x = 0, omega_accum_y = 0, omega_accum_z = 0;
    double dt = cur_time - prev_time;

    for(auto it = imu_buf.begin(); it != imu_buf.end(); ++it)
    {
        if(((*it)->header.stamp.toSec() < prev_time) && ((*(it+1))->header.stamp.toSec() > prev_time))
        {
            double cur_rx = (*(it))->angular_velocity.x;
            double cur_ry = (*(it))->angular_velocity.y;
            double cur_rz = (*(it))->angular_velocity.z;
            double next_rx = (*(it+1))->angular_velocity.x;
            double next_ry = (*(it+1))->angular_velocity.y;
            double next_rz = (*(it+1))->angular_velocity.z;
            double dt_1 = prev_time - (*it)->header.stamp.toSec();
            double dt_2 = (*(it+1))->header.stamp.toSec() - prev_time;
            double w1 = dt_2 / (dt_1 + dt_2);
            double w2 = dt_1 / (dt_1 + dt_2);
            double img_rx = w1 * cur_rx + w2 * next_rx;
            double img_ry = w1 * cur_ry + w2 * next_ry;
            double img_rz = w1 * cur_rz + w2 * next_rz;

            omega_accum_x += img_rx;
            omega_accum_y += img_ry;
            omega_accum_z += img_rz;
            n_omega_reading += 1;

            imu_buf.erase(it);
            it--;
        }
        else if(((*it)->header.stamp.toSec() > prev_time) && ((*it)->header.stamp.toSec() < cur_time) && ((*(it+1))->header.stamp.toSec() < cur_time))
        {
            double img_rx = (*(it))->angular_velocity.x;
            double img_ry = (*(it))->angular_velocity.y;
            double img_rz = (*(it))->angular_velocity.z;

            omega_accum_x += img_rx;
            omega_accum_y += img_ry;
            omega_accum_z += img_rz;
            n_omega_reading += 1;

            imu_buf.erase(it);
            it--;
        }
        else if(((*it)->header.stamp.toSec() > prev_time) && ((*it)->header.stamp.toSec() < cur_time) && ((*(it+1))->header.stamp.toSec() > cur_time))
        {
            double cur_rx = (*(it))->angular_velocity.x;
            double cur_ry = (*(it))->angular_velocity.y;
            double cur_rz = (*(it))->angular_velocity.z;
            double next_rx = (*(it+1))->angular_velocity.x;
            double next_ry = (*(it+1))->angular_velocity.y;
            double next_rz = (*(it+1))->angular_velocity.z;
            double dt_1 = cur_time - (*it)->header.stamp.toSec();
            double dt_2 = (*(it+1))->header.stamp.toSec() - cur_time;
            double w1 = dt_2 / (dt_1 + dt_2);
            double w2 = dt_1 / (dt_1 + dt_2);
            double img_rx = w1 * cur_rx + w2 * next_rx;
            double img_ry = w1 * cur_ry + w2 * next_ry;
            double img_rz = w1 * cur_rz + w2 * next_rz;

            omega_accum_x += img_rx;
            omega_accum_y += img_ry;
            omega_accum_z += img_rz;
            n_omega_reading += 1;
            break;
        }
        else
        {
            imu_buf.erase(it);
            it--;
        }
    }

    gyro_accum_x = omega_accum_x / n_omega_reading * dt;
    gyro_accum_y = omega_accum_y / n_omega_reading * dt;
    gyro_accum_z = omega_accum_z / n_omega_reading * dt;

    Matrix<float, 3, 1> gyro_accum_imu;
    gyro_accum_imu << gyro_accum_x, gyro_accum_y, gyro_accum_z;
    Matrix<float, 3, 3> extrinsicRotation_matrix;
    cv::cv2eigen(extrinsicRotation, extrinsicRotation_matrix);
    Matrix<float, 3, 1> gyro_accum_cam = extrinsicRotation_matrix.transpose() * gyro_accum_imu;

    ROS_INFO_STREAM("rx: " << gyro_accum_cam(0));
    ROS_INFO_STREAM("ry: " << gyro_accum_cam(1));
    ROS_INFO_STREAM("rz: " << gyro_accum_cam(2));

    cv::Mat r(3, 1, CV_32F);
    r.at<float>(0) = gyro_accum_cam(0);
    r.at<float>(1) = gyro_accum_cam(1);
    r.at<float>(2) = gyro_accum_cam(2);
    cv::Rodrigues(r, dR);

    return;
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

void OpticalFlowTracker::rejectByF()
{
    if(prev_pts.size() == 0)
        return;
    if(prev_pts.size() != cur_pts.size())
    {
        ROS_INFO_STREAM("rejectByF No Running!");
        return;
    }
    vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_prev_pts(prev_pts.size());
    //vector<cv::Point2f> un_cur_pts(), un_prev_pts();
    cv::Mat R = cv::Mat::eye(3,3,CV_32F);

    cv::undistortPoints(cur_pts, un_cur_pts, K, distortion_coeffs, R, K);
    cv::undistortPoints(prev_pts, un_prev_pts, K, distortion_coeffs, R, K);

    vector<uchar> status;
    cv::findFundamentalMat(un_cur_pts, un_prev_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);

    reduceVector<cv::Point2f>(status, cur_pts);
    reduceVector<cv::Point2f>(status, prev_pts);
    reduceVector<int>(status, ids);
    reduceVector<int>(status, track_cnt);
}

void OpticalFlowTracker::rejectByTwoPointRansac()
{
    if((prev_pts.size() != cur_pts.size()) || prev_pts.size() == 0)
    {
        ROS_INFO_STREAM("No Two Point Ransac!");
        return;
    }

    getImu();

    int num_points = prev_pts.size();

    Matrix<float, 3, Eigen::Dynamic> old_points(3, num_points);
    old_points.setZero();
    old_points.row(2) = Eigen::MatrixXf::Constant(1, num_points, 1); //将点矩阵的第三行设置为1

    Matrix<float, 3, Eigen::Dynamic> new_points(3, num_points);
    new_points.setZero();
    new_points.row(2) = Eigen::MatrixXf::Constant(1, num_points, 1); //将点矩阵的第三行设置为1

    Array<float, 2, 1> principal_point, focal_length;
    principal_point << K.at<float>(0, 2), K.at<float>(1, 2);
    focal_length << K.at<float>(0, 0), K.at<float>(1, 1);

    int col_count = 0;
    //将像素坐标转换为归一化平面上的坐标
    for(int i=0; i< num_points; i++)
    {
        Matrix<float, 2, 1> old_point;
        old_point << prev_pts[i].x, prev_pts[i].y;
        old_point.array() -= principal_point;
        old_point.array() /= focal_length;
        old_points.block(0, col_count, 2, 1) = old_point;

        Matrix<float, 2, 1> new_point;
        new_point << cur_pts[i].x, cur_pts[i].y;
        new_point.array() -= principal_point;
        new_point.array() /= focal_length;
        new_points.block(0, col_count++, 2, 1) = new_point;
    }

    int num_iters = 300; //设置迭代次数
    Array<bool, 1, Dynamic> status;
    int most_inliers = -1;

    //进入迭代
    for(int i=0; i<num_iters; i++)
    {
        //随机产生两个初始值
        int ind1 = rand() % num_points;
        int ind2 = rand() % num_points;
        while (ind1 == ind2)
            ind2 = rand() % num_points;

        Matrix<float, 3, 1> t; //定义平移向量

        const Matrix<float, 3, 1> p1 = new_points.col(ind1);
        const Matrix<float, 3, 1> p2 = new_points.col(ind2);
        //生成反对称矩阵
        Matrix<float, 3, 3> skewSymmetricP1, skewSymmetricP2;
        skewSymmetricP1 << 0, -p1(2), p1(1),
                        p1(2), 0, -p1(0),
                        -p1(1), p1(0), 0;
        skewSymmetricP2 << 0, -p2(2), p2(1),
                p2(2), 0, -p2(0),
                -p2(1), p2(0), 0;

        Matrix<float, 3, 3> dR_Matrix;
        cv::cv2eigen(dR, dR_Matrix);
        Matrix<float, 2, 3> M;
        M<<
         (dR_Matrix * old_points.col(ind1)).transpose() * skewSymmetricP1,
         (dR_Matrix * old_points.col(ind2)).transpose() * skewSymmetricP2;
        //求解平移向量
        if(!M.isZero(1e-9))
        {
            FullPivLU<Matrix<float, 2, 3>> lu_decomp(M);
            t = lu_decomp.kernel();
        }
        else
            t.setZero();
        if(t.cols() > 1)
            continue;

        //计算误差
        Matrix<float, 3, 3> skewSymmetricT;
        skewSymmetricT << 0, -t(2), t(1),
                t(2), 0, -t(0),
                -t(1), t(0), 0;
        Matrix<float, 3, 3> E = skewSymmetricT * dR_Matrix;

        //计算香农距离，多视图几何P222
        Array<float, 3, Dynamic> Ex1 = E * old_points;
        Array<float, 3, Dynamic> Ex2 = E.transpose() * old_points;
        Array<float, Dynamic, Dynamic> errs = ((new_points.array() * Ex1).colwise().sum()).square(); //colwise是取方向
        errs /=
                Ex1.row(0).array().square()+
                Ex1.row(1).array().square()+
                Ex2.row(0).array().square()+
                Ex2.row(1).array().square();
        Array<bool, 1, Dynamic> inliers = errs < RANSAC_THRESHOLD;
        int num_inliers = inliers.count();  //记录inliers中值为true的数量
        if(num_inliers > most_inliers)
        {
            most_inliers = num_inliers;
            status = inliers;
        }
    }

    reduceArray<cv::Point2f>(status, cur_pts);
    reduceArray<cv::Point2f>(status, prev_pts);
    reduceArray<int>(status, ids);
    reduceArray<int>(status, track_cnt);
}

void OpticalFlowTracker::readImage(const cv::Mat &_img, double _cur_time)
{
    cv::Mat img;
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    clahe->apply(_img, img);

    prev_time = cur_time;
    cur_time = _cur_time;
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
        //ROS_INFO_STREAM("prev_pts: " << prev_pts.size() <<" points");
        //cv::calcOpticalFlowPyrLK中点的坐标类型必须为单精度浮点数(float)
        cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21,21), 3);

        int count = 0;
        for(int i=0; i<int(cur_pts.size()); i++)
            if(status[i] && outImage(cur_pts[i]))
                status[i] = 0;

        reduceVector<cv::Point2f>(status, cur_pts);
        reduceVector<cv::Point2f>(status, prev_pts);
        reduceVector<int>(status, ids);
        reduceVector<int>(status, track_cnt);


        for(auto &n : track_cnt)
            ++n;
    }

    int n_add_cnt = MAX_CNT - static_cast<int>(cur_pts.size());
    if(n_add_cnt>20)
    {
        //rejectByTwoPointRansac();
        rejectByF();

        n_add_cnt = MAX_CNT - static_cast<int>(cur_pts.size());
        setMask();
        addPoints(n_add_cnt);
    }
    //如果不给参数R,K的话，去畸变函数给的是归一化坐标
    cv::undistortPoints(cur_pts, cur_un_pts, K, distortion_coeffs);
}

