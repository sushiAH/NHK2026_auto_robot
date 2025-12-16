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

    # SLAM用パラメータ
    slam_params = os.path.join(package_dir, "config", "slam_params.yaml")

    slam_package_dir = get_package_share_directory("slam_toolbox")
    print(slam_params)

    # laser_filter用パラメータ
    filter_params = os.path.join(package_dir, "config", "laser_filter.yaml")

    # ノード定義
    node1 = Node(
        package="auto_robot",  # package_name
        executable="odom_publisher",  # node_name
        output="screen",
    )

    node2 = Node(
        package="auto_robot",
        executable="twist_subscriber",
    )

    node3 = Node(
        package="joy_linux",
        executable="joy_linux_node",
    )

    node4 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
    )

    node5 = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params],
        remappings=[("/scan", "/scan_filtered")],
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

    # 静的tfの配信
    # base_link -> laser
    static_transform_publisher_laser_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["-0.3", "0.0", "0.0", "3.14", "0.0", "0.0", "base_link", "laser"],
    )

    # base_link -> base_footprint
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

    lidar_launch_file_dir = os.path.join(
        get_package_share_directory("sllidar_ros2"), "launch"
    )

    lidar_launch_file_path = os.path.join(lidar_launch_file_dir, "sllidar_s2_launch.py")

    lidar_setup_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_file_path),
    )

    ld.add_action(node1)
    ld.add_action(node2)
    ld.add_action(node3)
    ld.add_action(node4)
    ld.add_action(node5)
    ld.add_action(filter_node)
    ld.add_action(static_transform_publisher_laser_node)
    ld.add_action(static_transform_publisher_footprint_node)
    ld.add_action(lidar_setup_include)

    return ld
