import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import  PathJoinSubstitution

from launch_ros.actions import Node

def generate_launch_description():

    # setup paths
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_project_description = get_package_share_directory('robot_description')

    # load sdf file
    sdf_file = os.path.join(pkg_project_description, 'models','nomeer_robot','robot.sdf')
    with open(sdf_file, 'r') as infp:
        robot_dsc = infp.read()
    
    # launch gazebo 
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_description,
            'worlds',
            'tugbot_warehouse.sdf'
        ])}.items(),
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_description, 'config', 'ros_gz_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )
   # Rviz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='sim_rviz2',
        arguments=['-d' + os.path.join(pkg_project_description, 'rviz', 'rviz.rviz')]
    )
    return LaunchDescription([
        gz_sim,
        DeclareLaunchArgument('frame_id', default_value='odom', description='Frame ID of the parent frame'),
        DeclareLaunchArgument('child_frame_id', default_value='base_link', description='Frame ID of the child frame'),
        bridge,
        rviz_node,
        
        ])