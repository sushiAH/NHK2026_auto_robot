"""odom_publisher
twist_subscriber
joy_linux
rviz2
slam_toolbox
static tf lidar
static tf footprint
laser filter
scan Merger
pc to scan
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
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

    # 静的tfの配信
    # base_link -> laser
    static_tf_s2 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=[
            "0.0",
            "0.215",
            "0.0",
            "-1.5708",
            "0.0",
            "0.0",
            "base_link",
            "laser_s2",
        ],
    )

    static_tf_a3 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=[
            "0.0",
            "-0.215",
            "0.0",
            "1.5708",
            "0.0",
            "0.0",
            "base_link",
            "laser_a3",
        ],
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

    lidar_s2_launch_file_path = os.path.join(
        lidar_launch_file_dir, "sllidar_s2_launch.py"
    )

    lidar_a3_launch_file_path = os.path.join(
        lidar_launch_file_dir, "sllidar_a3_launch.py"
    )

    # --- RPLIDAR S2 の設定 ---
    lidar_s2_setup_include = GroupAction(
        actions=[
            # 1. 名前空間を設定（これでトピックが /lidar_s2/scan になります）
            PushRosNamespace("lidar_s2"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(lidar_s2_launch_file_path),
                launch_arguments={
                    "serial_port": "/dev/ttyUSB1",
                    "serial_baudrate": "1000000",  # S2の標準
                    "frame_id": "laser_s2",
                    "scan_mode": "",  # 10Hz(同期用)
                    "inverted": "false",
                }.items(),
            ),
        ]
    )

    # --- RPLIDAR A3 の設定 ---
    lidar_a3_setup_include = GroupAction(
        actions=[
            # 2. 名前空間を設定（これでトピックが /lidar_a3/scan になります）
            PushRosNamespace("lidar_a3"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(lidar_a3_launch_file_path),
                launch_arguments={
                    "serial_port": "/dev/ttyUSB2",
                    "serial_baudrate": "256000",  # A3の標準
                    "frame_id": "laser_a3",
                    "scan_mode": "Sensitivity",  # 10Hz(同期用)
                    "inverted": "false",
                }.items(),
            ),
        ]
    )

    # lidar統合node
    scan_merger_node = Node(
        package="ros2_laser_scan_merger",
        executable="ros2_laser_scan_merger",
        name="laser_scan_merger",
        parameters=[
            {
                "pointCloudTopic": "merged_cloud",
                "pointCloutFrameId": "base_link",
                "scanTopic1": "/lidar_s2/scan",
                "scanTopic2": "/lidar_a3/scan",
                "show1": True,
                "show2": True,
                "laser1Alpha": -90.0,
                "laser1XOff": 0.0,
                "laser1YOff": 0.125,
                "laser1ZOff": 0.0,
                "laser2Alpha": 90.0,
                "laser2XOff": 0.0,
                "laser2YOff": -0.125,
                "laser2ZOff": 0.0,
                "laser1AngleMax": 180.0,
                "laser1AngleMin": -180.0,
                "laser2AngleMax": 180.0,
                "laser2AngleMin": -180.0,
            }
        ],
    )

    # pointCloud -> scan
    pc_to_scan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        parameters=[
            {
                "target_frame": "base_link",
                "transform_tolerance": 0.01,
                "min_height": -1.0,
                "max_height": 1.0,
                "angle_min": -3.14159,
                "angle_max": 3.14159,
                "angle_increment": 0.0043,  # 約0.25度（LiDARの性能に合わせる）
                "scan_time": 0.1,
                "range_min": 0.2,
                "range_max": 30.0,
                "use_inf": True,
            }
        ],
        remappings=[
            ("cloud_in", "/merged_cloud"),  # Mergerの出力を入れる
            ("scan", "/scan_merged_raw"),  # フィルタ前のスキャンとして出力
        ],
    )

    # laser filter node
    # #Lidar(/scan) -> Filter ->/scan_filtered
    filter_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[filter_params, {"qos_overrides./scan.reliability": "best_effort"}],
        output="screen",
        remappings=[("scan", "/scan_merged_raw"), ("scan_filtered", "/scan_filtered")],
        #                 入力                         出力
    )

    ld.add_action(node1)
    ld.add_action(node2)
    ld.add_action(node3)
    ld.add_action(node4)
    ld.add_action(node5)

    ld.add_action(filter_node)

    ld.add_action(static_tf_s2)
    ld.add_action(static_tf_a3)

    ld.add_action(static_transform_publisher_footprint_node)

    ld.add_action(lidar_s2_setup_include)
    ld.add_action(lidar_a3_setup_include)

    ld.add_action(scan_merger_node)

    ld.add_action(pc_to_scan_node)

    return ld
