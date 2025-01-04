#pragma once

#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sensor_msgs/Image.h>

// Include opencv2
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Include CvBridge, Image Transport, Image msg
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <mutex>

#include <depth_img_normal_estimation/PinholeCamera.h>

class NormalEstimator
{
    public:

        NormalEstimator(ros::NodeHandle & nodeHandle);

        void runNormalEstimation();

    private:

        void depthImgCallback(const sensor_msgs::ImageConstPtr& msg);

        void estimateNormals(const cv::Mat& depth_img, cv::Mat& normals);

        image_transport::Subscriber depth_img_sub;
        std::mutex depth_img_mutex;
        ros::NodeHandle nodeHandle;
        sensor_msgs::ImageConstPtr depth_img_msg;

        PinholeCamera camera;
};