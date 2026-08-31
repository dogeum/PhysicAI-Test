from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('physicai_arm')
    
    urdf_path_default = os.path.join(pkg_share, "urdf", "physicai_arm.urdf")
    config_path_default = os.path.join(pkg_share, "config", "joints.yaml")
    
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    simulate = ParameterValue(LaunchConfiguration("simulate"), value_type=bool)
    config_path = LaunchConfiguration('config_path')
    
    with open(urdf_path_default, "r", encoding="utf-8") as f:
        robot_description = f.read()
    

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use /clock if true."
        ),
        DeclareLaunchArgument(
            "simulate",
            default_value="false",
            description="If true, do not access real hardware."
        ),
        DeclareLaunchArgument(
            "config_path",
            default_value=config_path_default,
            description="Path to joints.yaml"
        ),

        DeclareLaunchArgument(
            "enable_leader",
            default_value="false",
            description="Start leader arm driver if true."
        ),
        
        Node(
            package='physicai_arm',
            executable='feetech_follower_driver',
            name='feetech_follower_driver',
            output='screen',
            parameters=[
                {'config_path': config_path},
                {'simulate': simulate}
            ]
        ),
        
        Node(
            package='physicai_arm',
            executable='feetech_leader_driver',
            name='feetech_leader_driver',
            condition=IfCondition(LaunchConfiguration("enable_leader")),
            output='screen',
            parameters=[
                {'config_path': config_path},
                {'simulate': simulate}
            ]
        ),
        
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_base",
            arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
            output="screen"
        ),
        
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": use_sim_time
            }],
        ),
        
        Node(
            package="physicai_arm",
            executable="camera_node",
            name="arm_top_cam_publisher",
            output="screen",
            parameters=[{
                "camera_name": "top"
            }]
        ),

        Node(
            package="physicai_arm",
            executable="camera_node",
            name="arm_front_cam_publisher",
            output="screen",
            parameters=[{
                "camera_name": "front"
            }]
        ),
        
    ])
