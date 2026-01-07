import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    package_dir = get_package_share_directory("auto_robot")

    # slam_toolbox
    slam_params = os.path.join(package_dir, "config", "slam_params.yaml")
    slam_package_dir = get_package_share_directory("slam_toolbox")

    # nav2
    nav2_params_path = os.path.join(package_dir, "config", "nav2_params.yaml")
    map_file_path = ""

    filter_params = os.path.join(package_dir, "config", "laser_filter.yaml")

    print(slam_params)

    # setting node
    node1 = Node(
        package="auto_minicar",  # package_name
        executable="odom_publisher",  # node_name
        output="screen",
    )

    node2 = Node(
        package="auto_minicar",
        executable="twist_subscriber",
    )

    node3 = Node(
        package="joy_linux",
        executable="joy_linux_node",
    )

    node4 = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params],
    )

    # laser filter node
    # #Lidar(/scan) -> Filter ->/scan_filtered
    filter_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[filter_params],
        output="screen",
        remappings=[("scan", "/scan"), ("scan_filtered", "/scan_filtered")],
        #                 入力                         出力
    )

    static_transform_publisher_laser_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["-0.3", "0.0", "0.0", "3.14", "0.0", "0.0", "base_link", "laser"],
    )

    static_transform_publisher_footprint_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=[
            "0.0",
            "0.0",
            "0.0",
            "0.0",
            "0.0",
            "0.0",
            "base_link",
            "base_footprint",
        ],
    )

    # lidar
    lidar_launch_file_dir = os.path.join(
        get_package_share_directory("sllidar_ros2"), "launch"
    )

    lidar_launch_file_path = os.path.join(lidar_launch_file_dir, "sllidar_s2_launch.py")

    lidar_setup_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_file_path),
    )

    # nav2_launch_file
    nav2_launch_file_dir = os.path.join(
        get_package_share_directory("nav2_bringup"), "launch"
    )

    nav2_nav_launch_file_path = os.path.join(
        nav2_launch_file_dir, "navigation_launch.py"
    )

    nav2_nav_setup_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_nav_launch_file_path),
        launch_arguments={
            # "map": map_file_path,
            "params_file": nav2_params_path,
            "use_sim_time": "False",
            "autostart": "True",
        }.items(),
    )

    nav2_rviz2_launch_file_path = os.path.join(nav2_launch_file_dir, "rviz_launch.py")

    nav2_rviz2_setup_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_rviz2_launch_file_path),
    )

    ld.add_action(node1)
    ld.add_action(node2)
    # ld.add_action(node3)  # joy_linux
    # ld.add_action(node4)
    ld.add_action(static_transform_publisher_laser_node)
    ld.add_action(static_transform_publisher_footprint_node)
    ld.add_action(lidar_setup_include)
    ld.add_action(nav2_nav_setup_include)
    ld.add_action(nav2_rviz2_setup_include)
    ld.add_action(filter_node)

    return ld
