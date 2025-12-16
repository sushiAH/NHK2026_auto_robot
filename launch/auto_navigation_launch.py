import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # ==========================================
    # ユーザー設定エリア
    # ==========================================
    # 1. 地図ファイルの絶対パス
    map_file_path = (
        "/home/aratahorie/NHK2026_auto_robot/src/auto_robot/map/map_1765826754.yaml"
    )

    # 2. Nav2の設定ファイル
    nav2_params_path = (
        "/home/aratahorie/NHK2026_auto_robot/src/auto_robot/config/nav2_params.yaml"
    )

    # 3. フィルタの設定ファイル
    filter_config_path = (
        "/home/aratahorie/NHK2026_auto_robot/src/auto_robot/config/laser_filter.yaml"
    )

    # ==========================================

    # Nav2の起動ファイルディレクトリを取得
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    # publish static  tf
    static_transform_publisher_laser_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["-0.3", "0.0", "0.0", "3.14", "0.0", "0.0", "base_link", "laser"],
        parameters=[{"use_sim_time": False}],
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
        parameters=[{"use_sim_time": False}],
    )

    # lidar
    lidar_launch_file_dir = os.path.join(
        get_package_share_directory("sllidar_ros2"), "launch"
    )

    lidar_launch_file_path = os.path.join(lidar_launch_file_dir, "sllidar_s2_launch.py")

    lidar_setup_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_file_path),
    )

    # setting node
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

    # ---------------------------------------------------------
    # 1. フィルタノードの起動
    # ---------------------------------------------------------
    filter_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name="scan_to_scan_filter_chain",
        parameters=[filter_config_path],
        output="screen",
    )

    # ---------------------------------------------------------
    # 2. 地図と自己位置推定 (Localization) の起動
    # map_server と amcl をここで起動します
    # ---------------------------------------------------------
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "localization_launch.py")
        ),
        launch_arguments={
            "map": map_file_path,  # 地図パスを渡す
            "params_file": nav2_params_path,  # 設定ファイルを渡す
            "use_sim_time": "False",
            "autostart": "True",  # 自動でActiveにする
            "use_lifecycle_mgr": "True",  # 専用の管理人が立ち上がる
        }.items(),
    )

    # ---------------------------------------------------------
    # 3. ナビゲーション (Navigation) の起動
    # controller, planner, behavior server などを起動します
    # ---------------------------------------------------------
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "params_file": nav2_params_path,
            "use_sim_time": "False",
            "autostart": "True",
            "use_lifecycle_mgr": "True",
        }.items(),
    )

    return LaunchDescription(
        [
            filter_node,
            localization_launch,
            navigation_launch,
            node1,
            node2,
            node3,
            lidar_setup_include,
            static_transform_publisher_laser_node,
            static_transform_publisher_footprint_node,
        ]
    )
