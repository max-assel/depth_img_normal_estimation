#include <depth_img_normal_estimation/NormalEstimator.h>

NormalEstimator::NormalEstimator(ros::NodeHandle & nodeHandle)
{
    image_transport::ImageTransport it(nodeHandle);

    // Initialize subscriber
    depth_img_sub = it.subscribe("/camera/depth/image", 1, &NormalEstimator::depthImgCallback, this);
}

void NormalEstimator::depthImgCallback(const sensor_msgs::ImageConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(depth_img_mutex);
    depth_img_msg = msg;
}

void NormalEstimator::runNormalEstimation()
{
    std::lock_guard<std::mutex> lock(depth_img_mutex);

    // Read depth image
    cv::Mat depth_img = cv_bridge::toCvShare(depth_img_msg)->image; // , "32FC1"

    // Normals
    cv::Mat normals;
    estimateNormals(depth_img, normals);

    return;
}

void NormalEstimator::estimateNormals(const cv::Mat& depth_img, cv::Mat& normals)
{
    // Normal estimation code

    int rows = depth_img.rows;
    int cols = depth_img.cols;

    for (int r = 0; r < (rows - 1); r++)
    {
        for (int c = 0; c < (cols - 1); c++)
        {
            // Do something
            Z = depth_img.at<float>(r, c);
            Z_r = depth_img.at<float>(r + 1, c);
            Z_c = depth_img.at<float>(r, c + 1);

            // Calculate depth gradient
            dZ_dx = (Z_c - Z);
            dZ_dy = (Z_r - Z);

            // Calculate X/Y gradients
            dX_dx = (Z / camera.fx) + dZ_dx * (c - camera.u_0) / camera.fx;
            dY_dx = dZ_dx * (r - camera.v_0) / camera.fy;

            dX_dy = dZ_dy * (c - camera.u_0) / camera.fx;
            dY_dy = (Z / camera.fy) + dZ_dy * (r - camera.v_0) / camera.fy;

            // Calculate direcitonal derivatives
            Eigen::Vector3f v_x(dX_dx, dY_dx, dZ_dx);
            Eigen::Vector3f v_y(dX_dy, dY_dy, dZ_dy);

            // Calculate normal
            Eigen::Vector3f n = v_y.cross(v_x); // I think?

        }
    }

    // take penultimate row/col and copy to last row/col


    return;
}