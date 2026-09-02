from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('physicai_arm')
    
    urdf_path_default = os.path.join(pkg_share, "urdf", "physicai_arm.urdf")
    
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    simulate = ParameterValue(LaunchConfiguration("simulate"), value_type=bool)
    config_path = LaunchConfiguration('config_path')
    
    with open(urdf_path_default, "r", encoding="utf-8") as f:
        robot_description = f.read()

    home_wrist_roll = ParameterValue(
        LaunchConfiguration("home_wrist_roll_rad"), value_type=float
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            "home_wrist_roll_rad",
            default_value="1.5708",
            description=(
                "Absolute wrist_roll angle (rad) the IK holds as home. "
                "1.5708 = +90 deg (CCW); use -1.5708 to flip direction, "
                "or 'nan' to disable and keep the measured roll."
            ),
        ),

        Node(
            package="joy",
            executable="joy_node",
            name="joystick_pub_node",
            output="screen",
            parameters=[{
                "device": "/dev/input/js0"
            }]
        ),

        Node(
            package="physicai_arm",
            executable="ik_calc",
            name="ik_inference",
            output="screen",
            parameters=[{
                "home_wrist_roll_rad": home_wrist_roll
            }]
        ),

        # Publishes /ee_pose from /robot_description + /follower/joint_states
        # (both provided by bringup.launch.py). joy_to_target uses it to snap
        # its internal EE target to the arm's real pose on start-up.
        Node(
            package="physicai_arm",
            executable="fk_calc",
            name="fk_inference",
            output="screen"
        ),

        Node(
            package="physicai_arm",
            executable="joy_to_target",
            name="joystick_bridge_node",
            output="screen"
        )

    ])
