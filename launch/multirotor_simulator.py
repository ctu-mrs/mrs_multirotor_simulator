#!/usr/bin/env python3

import launch
import os
import sys

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    IfElseSubstitution,
    PythonExpression,
    PathJoinSubstitution,
    EnvironmentVariable,
)

from ament_index_python.packages import get_package_share_directory

def load_custom_config(name, param_file_list = None):

    if param_file_list == None:
        param_file_list = []

    # custom config for param server
    custom_config=os.getenv(name)

    if custom_config:
        param_file_list = param_file_list + [os.path.abspath(custom_config)]

    return param_file_list

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "mrs_multirotor_simulator"

    pkg_share_path = get_package_share_directory(pkg_name)
    namespace='multirotor_simulator'

    param_files = load_custom_config("custom_config")

    ld.add_action(ComposableNodeContainer(

        namespace='',
        name=namespace+'_container',
        package='rclcpp_components',

        executable='component_container_mt',

        # prefix=['debug_roslaunch ' + os.ttyname(sys.stdout.fileno())],

        composable_node_descriptions=[

            ComposableNode(

                package=pkg_name,
                plugin='mrs_multirotor_simulator::MultirotorSimulator',
                namespace='',
                name='multirotor_simulator',

                parameters=[
                    {'config': pkg_share_path + '/config/multirotor_simulator.yaml'},
                    {'config_uavs': pkg_share_path + '/config/uavs.yaml'},
                    {'custom_config': ''},
                    pkg_share_path + '/config/controllers/attitude_controller.yaml',
                    pkg_share_path + '/config/controllers/rate_controller.yaml',
                    pkg_share_path + '/config/controllers/position_controller.yaml',
                    pkg_share_path + '/config/controllers/velocity_controller.yaml',
                    pkg_share_path + '/config/controllers/mixer.yaml',
                    pkg_share_path + '/config/uavs/x500.yaml',
                ],

                # remappings=[
                #     ("~/topic", "~/topic"),
                # ],
            )

        ],

        output='screen',

    ))

    return ld
