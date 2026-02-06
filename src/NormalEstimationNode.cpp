#include <depth_img_normal_estimation/NormalEstimator.h>

int main(int argc, char** argv) 
{
    // Initialize ros node
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr nodePtr = rclcpp::Node::make_shared("depth_img_normal_estimation_node",
                                                                rclcpp::NodeOptions()
                                                                .allow_undeclared_parameters(true)
                                                                .automatically_declare_parameters_from_overrides(true));

    rclcpp::Rate loop_rate(30);

    std::string camera_normals_topic = nodePtr->get_parameter("camera_normals_topic").as_string();

    std::string camera_depth_topic = nodePtr->get_parameter("camera_depth_topic").as_string();

    std::string config_path = nodePtr->get_parameter("config_path").as_string();

    bool hardware = nodePtr->get_parameter("hardware").as_bool();

    // Create NormalEstimator object
    NormalEstimator normalEstimator(nodePtr, camera_depth_topic, camera_normals_topic, config_path, hardware);

    // rclcpp::spin(nodePtr);

    while (rclcpp::ok())
    {
        normalEstimator.runNormalEstimation();

        loop_rate.sleep();

        rclcpp::spin_some(nodePtr);
    }

}