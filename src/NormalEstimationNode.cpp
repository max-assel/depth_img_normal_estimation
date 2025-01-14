#include <ros/init.h>
#include <ros/node_handle.h>

#include <depth_img_normal_estimation/NormalEstimator.h>

void ros_throw_if(const bool & condition, const std::string & message)
{
    if (condition)
    {
        ROS_ERROR_STREAM(message);
        throw std::runtime_error(message);
    }
}

void ros_throw_param_load(const ros::NodeHandle & nh, const std::string & param_name, std::string & param)
{
    return ros_throw_if( !nh.getParam(param_name, param), "Couldn't find parameter: " + param_name);
}

void ros_throw_param_load(const ros::NodeHandle & nh, const std::string & param_name, bool & param)
{
    return ros_throw_if( !nh.getParam(param_name, param), "Couldn't find parameter: " + param_name);
}

int main(int argc, char** argv) 
{
    // Initialize ros node
    ros::init(argc, argv, "normal_estimation_node");
    ros::NodeHandle nodeHandle;

    ros::Rate loop_rate(30);

    std::string camera_depth_topic;
    ros_throw_param_load(nodeHandle, "/camera_depth_topic", camera_depth_topic);

    std::string config_path;
    ros_throw_param_load(nodeHandle, "/config_path", config_path);

    bool hardware;
    ros_throw_param_load(nodeHandle, "/hardware", hardware);

    // Create NormalEstimator object
    NormalEstimator normalEstimator(nodeHandle, camera_depth_topic, config_path, hardware);

    ros::spin();

    // while (ros::ok())
    // {
    //     // Do something
    //     normalEstimator.runNormalEstimation();
     
    //     loop_rate.sleep();
    //     ros::spinOnce();
    // }

}