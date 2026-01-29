import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    realsense2_camera_path = get_package_share_directory("realsense2_camera")

    ld = launch.LaunchDescription([
        launch_ros.actions.SetParameter(name='use_sim_time', value=False),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false", description="Use simulation (Gazebo) clock if true"
        ),        
        launch_ros.actions.Node(
            package='depth_img_normal_estimation',
            executable='depth_img_normal_estimation_node',
            name='depth_img_normal_estimation_node',
            output='screen',
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration("use_sim_time")
                },
                {
                    'camera_depth_topic': '/D435/depth/image_rect_raw'
                },
                {
                    'camera_normals_topic': '/D435/normals'
                },
                {
                    'config_path': get_package_share_directory('depth_img_normal_estimation') + '/cfg/hardware.yaml'
                },
                {
                    'hardware': True
                }
            ]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    realsense2_camera_path, "launch", "rs_d435_launch.py",
                )
            ),
            launch_arguments={
                    'use_sim_time': LaunchConfiguration("use_sim_time")
            }.items()
        ),
        launch_ros.actions.Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            # arguments=[
            #     "-d",
            #     os.path.join(
            #         anymal_interface_path, "rviz", "anymal_mmp.rviz",
            #     )
            # ],
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration("use_sim_time")
                }
            ]
        )        

    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
