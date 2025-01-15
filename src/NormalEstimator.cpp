#include <depth_img_normal_estimation/NormalEstimator.h>

NormalEstimator::NormalEstimator(ros::NodeHandle & nodeHandle, 
                                    const std::string & camera_depth_topic, 
                                    const std::string & config_path,
                                    const bool & hardware)
{
    image_transport::ImageTransport it(nodeHandle);

    // Initialize subscriber
    depth_img_sub = it.subscribe(camera_depth_topic, 1, &NormalEstimator::depthImgCallback, this);

    // Initialize publishers
    normals_pub = it.advertise("/camera/normals", 1);
    normals_bgr_img_pub = it.advertise("/camera/color_normals", 1);
    filtered_depth_pub = it.advertise("/camera/depth/filtered", 1);

    // Load configs
    YAML::Node configYamlNode = YAML::LoadFile(config_path);

    params.depth_thresh = configYamlNode["normal_estimation"]["depth_threshold"].as<float>();
    params.bilat_filter_num_iters = configYamlNode["bilateral_filter"]["num_iters"].as<int>();
    params.bilat_filter_kernel_size = configYamlNode["bilateral_filter"]["kernel_size"].as<int>();
    params.bilat_filter_sigma_color = configYamlNode["bilateral_filter"]["sigma_color"].as<float>();
    params.bilat_filter_sigma_space = configYamlNode["bilateral_filter"]["sigma_space"].as<float>();
    params.infill_filter_kernel_size = configYamlNode["infill_filter"]["kernel_size"].as<int>();

    hardware_ = hardware;

    filtered_depth_ptr.reset(new cv_bridge::CvImage);
    normals_ptr.reset(new cv_bridge::CvImage);
    normals_bgr_ptr.reset(new cv_bridge::CvImage);
}

void NormalEstimator::depthImgCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    // std::lock_guard<std::mutex> lock(depth_img_mutex);

    // ROS_INFO("[NormalEstimator::depthImgCallback]");

    try
    {
        depth_img_ptr = cv_bridge::toCvCopy(msg, "32FC1");
        // ROS_INFO("      received new depth image");
    } catch (std::exception& e)
    {
        ROS_ERROR("       depthImgCallback failed: %s", e.what());
        return;
    }       

    runNormalEstimation();

}

bool NormalEstimator::notReceivedDepthImage()
{
    return depth_img_ptr == nullptr;
}

void NormalEstimator::runNormalEstimation()
{
    // std::lock_guard<std::mutex> lock(depth_img_mutex);

    if (notReceivedDepthImage())
    {
        ROS_WARN("Not ready to estimate normals, no depth image received yet.");
        return;
    }

    // Read depth image
    cv::Mat depth_img = depth_img_ptr->image;
    int rows = depth_img.rows;
    int cols = depth_img.cols;

    // Pre-process depth image
    cv::Mat depth_img_preprocessed;
        
    if (hardware_)
    {
        // bilateral filter
        for (int i = 0; i < params.bilat_filter_num_iters; i++)
        {
            cv::Mat temp_img; 
            cv::bilateralFilter(depth_img, 
                                temp_img, 
                                params.bilat_filter_kernel_size, 
                                params.bilat_filter_sigma_color, 
                                params.bilat_filter_sigma_space);
            depth_img = temp_img;
        }


        // shadow infill
        cv::Mat shadow_infill_kernel = cv::Mat::ones(params.infill_filter_kernel_size, params.infill_filter_kernel_size, CV_32F);

        cv::dilate(depth_img, depth_img_preprocessed, shadow_infill_kernel);

    } else
    {
        depth_img_preprocessed = depth_img.clone();
        // cv::bilateralFilter(depth_img, 
        //                     depth_img_preprocessed, 
        //                     params.bilat_filter_kernel_size, 
        //                     params.bilat_filter_sigma_color, 
        //                     params.bilat_filter_sigma_space);
    }

    // Publish filtered depth image
    // cv_bridge::CvImagePtr filtered_depth_ptr(new cv_bridge::CvImage);
    filtered_depth_ptr->header = depth_img_ptr->header;
    filtered_depth_ptr->encoding = "32FC1";
    filtered_depth_ptr->image = depth_img_preprocessed;

    filtered_depth_pub.publish(filtered_depth_ptr->toImageMsg());

    // Normals
    // set size as h x w x 3
    // cv_bridge::CvImagePtr normals_ptr(new cv_bridge::CvImage);
    normals_ptr->header = depth_img_ptr->header;                                
    normals_ptr->encoding = "32FC3";                                            
    normals_ptr->image = cv::Mat(depth_img.rows, depth_img.cols, CV_32FC3, cv::Scalar(0.0, 0.0, 0.0));

    estimateNormals(depth_img_preprocessed, normals_ptr);

    // Query middle normal
    // int r = rows / 2;
    // int c = cols / 2;
    // ROS_INFO("      middle normal (%i, %c): %f %f %f", r, c, normals_ptr->image.at<cv::Vec3f>(r, c)[0], 
    //                                                             normals_ptr->image.at<cv::Vec3f>(r, c)[1], 
    //                                                             normals_ptr->image.at<cv::Vec3f>(r, c)[2]);

    // cv_bridge::CvImagePtr normals_bgr_ptr(new cv_bridge::CvImage);
    normals_bgr_ptr->header = depth_img_ptr->header;                                
    normals_bgr_ptr->encoding = "rgb8";                                            
    normals_bgr_ptr->image = normals_ptr->image.clone();

    // Convert from float to 8UC3
    // First, take abs value of normals
    normals_bgr_ptr->image = cv::abs(normals_bgr_ptr->image);

    // Then, convert to 8UC3
    normals_bgr_ptr->image.convertTo(normals_bgr_ptr->image, CV_8UC3, 255.0);

    // ROS_INFO("      middle colored normal: %d %d %d", normals_bgr_ptr->image.at<cv::Vec3b>(r, c)[0], 
    //                                                     normals_bgr_ptr->image.at<cv::Vec3b>(r, c)[1], 
    //                                                     normals_bgr_ptr->image.at<cv::Vec3b>(r, c)[2]);

    // Display normals
    publishNormals(normals_ptr, normals_bgr_ptr);

    return;
}

void NormalEstimator::publishNormals(const cv_bridge::CvImagePtr& normals, const cv_bridge::CvImagePtr& normals_bgr)
{
    normals_pub.publish(normals->toImageMsg());
    normals_bgr_img_pub.publish(normals_bgr->toImageMsg());
}

void NormalEstimator::estimateNormals(const cv::Mat& depth_img, cv_bridge::CvImagePtr& normals)
{
    // ROS_INFO("[NormalEstimator::estimateNormals]");

    float scale = 0.001; // 1000;

    // Normal estimation code

    int rows = depth_img.rows;
    int cols = depth_img.cols;

    float Z = 0.0, Z_r = 0.0, Z_c = 0.0;
    float dZ_dx = 0.0, dZ_dy = 0.0;
    float dX_dx = 0.0, dY_dx = 0.0;
    float dX_dy = 0.0, dY_dy = 0.0;
    Eigen::Vector3f v_x, v_y, n;

    int row_print = rows - 2;
    int col_print = cols / 2;

    for (int r = 0; r < (rows - 1); r++)
    {
        for (int c = 0; c < (cols - 1); c++)
        {
            // ROS_INFO("      pixel: (%d, %d)", r, c);

            // Do something
            Z = depth_img.at<float>(r, c) * scale;
            Z_r = depth_img.at<float>(r + 1, c) * scale;
            Z_c = depth_img.at<float>(r, c + 1) * scale;

            // Calculate depth gradient
            dZ_dx = (Z_c - Z);
            dZ_dy = (Z_r - Z);

            if (std::abs(dZ_dx) > params.depth_thresh || std::abs(dZ_dy) > params.depth_thresh)
                continue;

            // Calculate X/Y gradients
            dX_dx = (Z / camera.fx) + dZ_dx * (c - camera.u_0) / camera.fx;
            dY_dx = dZ_dx * (r - camera.v_0) / camera.fy;

            dX_dy = dZ_dy * (c - camera.u_0) / camera.fx;
            dY_dy = (Z / camera.fy) + dZ_dy * (r - camera.v_0) / camera.fy;

            // Calculate direcitonal derivatives
            v_x << dX_dx, dY_dx, dZ_dx;
            v_y << dX_dy, dY_dy, dZ_dy;

            // Calculate normal
            n = v_y.cross(v_x); // I think it should be y cross x

            // ROS_INFO("      raw normal: %f %f %f", n(0), n(1), n(2));

            // Normalize normal
            n.normalize();

            // if (r == row_print && c == col_print)
            // {
            //     ROS_INFO("      pixel: (%d, %d)", r, c);
                
            //     ROS_INFO("      Z: %f", Z);
            //     ROS_INFO("      Z_r: %f", Z_r);
            //     ROS_INFO("      Z_c: %f", Z_c);

            //     ROS_INFO("      dZ_dx: %f", dZ_dx);
            //     ROS_INFO("      dZ_dy: %f", dZ_dy);

            //     ROS_INFO("      dX_dx: %f", dX_dx);
            //     ROS_INFO("      dY_dx: %f", dY_dx);
            //     ROS_INFO("      dX_dy: %f", dX_dy);
            //     ROS_INFO("      dY_dy: %f", dY_dy);    

            //     ROS_INFO("      v_x: %f %f %f", v_x(0), v_x(1), v_x(2));
            //     ROS_INFO("      v_y: %f %f %f", v_y(0), v_y(1), v_y(2));

            //     ROS_INFO("      normal: %f %f %f", n(0), n(1), n(2));
            // }

            // Set normal
            // normals->image.at<cv::Vec3b>(r, c)[2] = int(255 * std::abs(n(0))); // taking abs just to ensure RGB values are positive
            // normals->image.at<cv::Vec3b>(r, c)[1] = int(255 * std::abs(n(1))); // taking abs just to ensure RGB values are positive
            // normals->image.at<cv::Vec3b>(r, c)[0] = int(255 * std::abs(n(2))); // taking abs just to ensure RGB values are positive
            normals->image.at<cv::Vec3f>(r, c)[0] = n(0);
            normals->image.at<cv::Vec3f>(r, c)[1] = n(1);
            normals->image.at<cv::Vec3f>(r, c)[2] = n(2);
        }
    }

    // take penultimate row/col and copy to last row/col
    normals->image.row(rows - 1) = normals->image.row(rows - 2).clone();
    normals->image.col(cols - 1) = normals->image.col(cols - 2).clone();

    // for (int c = 0; c < cols; c++)
    // {
    //     normals->image.at<cv::Vec3b>(rows - 1, c)[0] = normals->image.at<cv::Vec3b>(rows - 2, c)[0];
    //     normals->image.at<cv::Vec3b>(rows - 1, c)[1] = normals->image.at<cv::Vec3b>(rows - 2, c)[1];
    //     normals->image.at<cv::Vec3b>(rows - 1, c)[2] = normals->image.at<cv::Vec3b>(rows - 2, c)[2];
    // }

    // for (int r = 0; r < rows; r++)
    // {
    //     normals->image.at<cv::Vec3b>(r, cols - 1)[0] = normals->image.at<cv::Vec3b>(r, cols - 2)[0];
    //     normals->image.at<cv::Vec3b>(r, cols - 1)[1] = normals->image.at<cv::Vec3b>(r, cols - 2)[1];
    //     normals->image.at<cv::Vec3b>(r, cols - 1)[2] = normals->image.at<cv::Vec3b>(r, cols - 2)[2];
    // }

    return;
}