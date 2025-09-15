# 匯入 ROS2 啟動相關模組
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # 取得 r_go2_description 與 go2_driver 套件的分享目錄
    r_go2_desc_pkg = get_package_share_directory("r_go2_description")
    go2_driver_pkg = get_package_share_directory("go2_driver_py")

    # 宣告是否啟動 RViz2 的參數
    use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="是否啟動 RViz2"
    )

    # 回傳 LaunchDescription，包含所有啟動項目
    return LaunchDescription([
        use_rviz,
        # 載入 r_go2_description 的 display.launch.py，顯示機器人模型
        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(
                launch_file_path=os.path.join(r_go2_desc_pkg, "launch", "display.launch.py")
            ),
            launch_arguments = [("use_joint_state_publisher", "false")]
        ),

        # 啟動 RViz2 節點，載入指定的 rviz 配置檔
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", os.path.join(go2_driver_pkg, "rviz", "display.rviz")],
            condition=IfCondition(LaunchConfiguration("use_rviz"))
        ),

        # 啟動靜態 TF 發佈器，建立 radar 與 utlidar_lidar 之間的靜態座標轉換
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["--frame-id", "radar", "--child-frame-id", "utlidar_lidar"]
        ),

        # 啟動 go2_twist_bridge 節點，負責速度指令橋接
        Node(
            package="go2_twist_bridge",
            executable="twist_bridge",
        ),

        # 啟動 go2_driver 節點，負責底層驅動
        Node(
            package="go2_driver_py",
            executable="driver",
            parameters=[os.path.join(go2_driver_pkg, "params", "driver.yaml")]
        ),
    ])