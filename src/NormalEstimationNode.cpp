#include <ros/init.h>
#include <ros/node_handle.h>

#include <depth_img_normal_estimation/NormalEstimator.h>

int main(int argc, char** argv) 
{
    // Initialize ros node
    ros::init(argc, argv, "normal_estimation_node");
    ros::NodeHandle nodeHandle;

    // Create NormalEstimator object
    NormalEstimator normalEstimator(nodeHandle);

    ros::Rate loop_rate(30);

    while (ros::ok())
    {
        // Do something
        normalEstimator.runNormalEstimation();
     
        loop_rate.sleep();
        ros::spinOnce();
    }


}