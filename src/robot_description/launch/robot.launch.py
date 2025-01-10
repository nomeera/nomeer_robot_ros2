import os  # Provides utilities for handling file paths and interacting with the file system.
from ament_index_python.packages import (
    get_package_share_directory,
)  # Retrieves the share directory of a ROS 2 package.
import xml.etree.ElementTree as ET

# these modules define the structure and actions of the launch file.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

# Used to define and launch ROS 2 nodes
from launch_ros.actions import Node

#  modify_sdf_file to fix ball joint error appear in rviz
def modify_sdf_file(sdf_file):
    # Parse the SDF XML
    tree = ET.parse(sdf_file)
    root = tree.getroot()

    # Find all joints in the SDF
    for joint in root.findall(".//joint"):
        joint_type = joint.get("type")
        if joint_type == "ball":
            print(f"Changing joint '{joint.get('name')}' from 'ball' to 'fixed'")
            joint.set("type", "fixed")

    # Write back to a temporary file
    modified_sdf_file = sdf_file.replace(".sdf", "_modified.sdf")
    tree.write(modified_sdf_file)
    return modified_sdf_file

def generate_launch_description():

    # setup paths
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_project_description = get_package_share_directory("robot_description")

    # load sdf file
    sdf_file = os.path.join(
        pkg_project_description, "models", "nomeer_robot", "robot.sdf"
    )
    modified_sdf_file = modify_sdf_file(sdf_file)
    with open(modified_sdf_file, "r") as infp:
        robot_desc = infp.read()
          # Reads the content of the SDF file into a string for use in other components.

    # launch gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_ros_gz_sim, "launch", "gz_sim.launch.py"
            )  # Includes another launch file to start the Gazebo simulation.
        ),
        launch_arguments={
            "gz_args": PathJoinSubstitution(
                [pkg_project_description, "worlds", "industrial-warehouse.sdf"]
            )
        }.items(),
    )

    # joint state publisher to publish joint state
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="both",
        parameters=[
            {"use_sim_time": True},
        ],
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"use_sim_time": True},
            {"robot_description": robot_desc},
        ],
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": os.path.join(
                    pkg_project_description, "config", "ros_gz_bridge.yaml"
                ),
                "qos_overrides./tf_static.publisher.durability": "transient_local",  # Overrides the QoS (Quality of Service) settings for the /tf_static topic to make it durable (e.g., for transformations).
            }
        ],
        output="screen",
    )

    # rviz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        name="sim_rviz2",
        arguments=['-d', 'src/robot_description/rviz/rviz.rviz']    
        )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        joint_state_publisher,
        bridge, 
        rviz_node
        ])
