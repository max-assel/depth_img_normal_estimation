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

        void depthImgCallback(const sensor_msgs::Image::ConstPtr& msg);

        void estimateNormals(const cv::Mat& depth_img, cv_bridge::CvImagePtr& normals);

        void publishNormals(const cv_bridge::CvImagePtr& normals);

        bool readyToEstimateNormals();

        image_transport::Subscriber depth_img_sub;
        image_transport::Publisher normals_img_pub; /**< estimated normals image publisher */

        std::mutex depth_img_mutex;
        ros::NodeHandle nodeHandle;

        cv_bridge::CvImagePtr depth_img_ptr;


        PinholeCamera camera;
};