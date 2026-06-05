"""
Optimized Gazebo Launch File for Rover Simulation

Eliminates clock synchronization errors and runtime issues using timer-based sequencing.
All nodes use consistent simulation time to prevent timestamp conflicts.
"""

import os
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription, 
    SetEnvironmentVariable,
    TimerAction
)
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Get package directory and detect ROS distribution
    rover_description = get_package_share_directory("rover_description")
    ros_distro = os.environ.get("ROS_DISTRO")
    is_ignition = "True" if ros_distro == "humble" else "False"

    # ========== LAUNCH ARGUMENTS ==========
    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use simulation time. Must be true for Gazebo."
    )

    is_sim_arg = DeclareLaunchArgument(
        name="is_sim",
        default_value="true",
        description="Start robot in simulation mode"
    )
    
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(rover_description, "urdf", "rover_description.urdf.xacro"),
        description="Absolute path to robot urdf file"
    )

    world_name_arg = DeclareLaunchArgument(
        name="world_name", 
        default_value="empty",
        description="Gazebo world name (without extension)"
    )
    
    world_extension_arg = DeclareLaunchArgument(
        name='world_extension',
        default_value='.world',
        description='World file extension (.world or .sdf)'
    )
    
    # Construct full world file path
    world_path = PathJoinSubstitution([
        rover_description,
        "worlds",
        PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'", " + '", LaunchConfiguration("world_extension"), "'"])
    ])

    # ========== ENVIRONMENT SETUP ==========
    model_path = str(Path(rover_description).parent.resolve())
    model_path += pathsep + os.path.join(rover_description, 'models')

    gazebo_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        model_path
    )
    
    # ========== ROBOT DESCRIPTION ==========
    # Process xacro file to generate URDF
    robot_description = ParameterValue(
        Command([
            'xacro ', 
            LaunchConfiguration('model'), 
            ' is_ignition:=', is_ignition,
            ' is_sim:=', LaunchConfiguration('is_sim'),
            ' use_sim_time:=', LaunchConfiguration('use_sim_time')
        ]), 
        value_type=str
    )
    
    # ========== CORE NODES ==========
    
    # Robot State Publisher - publishes /robot_description and TF
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": LaunchConfiguration('use_sim_time')
        }],
        output="screen"
    )

    # Gazebo Simulator
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
        launch_arguments={
            "gz_args": PythonExpression(["'", world_path, " -v 4 -r'"])
        }.items()
    )
    
    # Clock Bridge - CRITICAL: provides /clock topic for simulation time
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
        ],
        output="screen",
    )
    
    # Entity Spawner - spawns robot in Gazebo
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        name="gz_spawn_entity",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "rover",
        ],
        parameters=[{"use_sim_time": LaunchConfiguration('use_sim_time')}],
    )
    
    # ========== SENSOR BRIDGES ==========
    # Bridge Gazebo sensor topics to ROS
    
    sensor_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="sensor_bridge",
        arguments=[
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
        ],
        remappings=[
            ('/imu', '/imu/out'),
        ],
        parameters=[{"use_sim_time": LaunchConfiguration('use_sim_time')}],
        output="screen",
    )
    
    image_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="image_bridge",
        arguments=[
            "/zed/zed_node/rgb/image_rect_color@sensor_msgs/msg/Image[gz.msgs.Image"
        ],
        parameters=[{"use_sim_time": LaunchConfiguration('use_sim_time')}],
        output="screen",
    )
    
    depth_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="depth_bridge",
        arguments=[
            "/zed/zed_node/depth/depth_registered@sensor_msgs/msg/Image[gz.msgs.Image"
        ],
        parameters=[{"use_sim_time": LaunchConfiguration('use_sim_time')}],
        output="screen",
    )
    
    camera_info_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_info_bridge",
        arguments=[
            "/zed/zed_node/rgb/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
        ],
        parameters=[{"use_sim_time": LaunchConfiguration('use_sim_time')}],
        output="screen",
    )
    
    depth_camera_info_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="depth_camera_info_bridge",
        arguments=[
            "/zed/zed_node/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
        ],
        parameters=[{"use_sim_time": LaunchConfiguration('use_sim_time')}],
        output="screen",
    )
    
    point_cloud_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="point_cloud_bridge",
        arguments=[
            "/zed/zed_node/depth/depth_registered/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"
        ],
        parameters=[{"use_sim_time": LaunchConfiguration('use_sim_time')}],
        output="screen",
    )

    # ========== TIMER-BASED SEQUENCING ==========
    # Using absolute timers from launch start
    
    # Spawn entity 5s after launch (allows time for robot_description topic)
    delayed_spawn = TimerAction(
        period=5.0,
        actions=[gz_spawn_entity]
    )
    
    # Start sensor bridges 8s after launch (2-3s after entity spawn)
    delayed_sensors = TimerAction(
        period=8.0,
        actions=[
            sensor_bridge,
            image_bridge,
            depth_bridge,
            camera_info_bridge,
            depth_camera_info_bridge,
            point_cloud_bridge,
        ]
    )

    # ========== LAUNCH DESCRIPTION ==========
    return LaunchDescription([
        # Arguments
        model_arg,
        world_name_arg,
        is_sim_arg,
        use_sim_time_arg,
        world_extension_arg,

        # Environment
        gazebo_resource_path,
        
        # Launch sequence (all times from t=0)
        gazebo,                      # t=0s: Gazebo starts
        clock_bridge,                # t=0s: Clock bridge starts with Gazebo
        robot_state_publisher_node,  # t=0s: Publish robot_description
        delayed_spawn,               # t=5s: Spawn robot entity
        delayed_sensors,             # t=8s: Start sensor bridges
    ])