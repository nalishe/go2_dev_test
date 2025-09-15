# 匯入 ROS2 啟動相關模組
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument

# 用於取得套件分享目錄
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # 取得 r_go2_description 套件的分享目錄
    r_go2_decription_pkg = get_package_share_directory("r_go2_description")

    # 宣告 urdf_path 參數，預設為 go2_description.urdf
    model = DeclareLaunchArgument(
        name="urdf_path",
        default_value=os.path.join(r_go2_decription_pkg, "urdf", "go2_description.urdf")
    )
    # 宣告是否啟用 joint_state_publisher，預設為 true
    use_joint_state_publisher = DeclareLaunchArgument(
        name="use_joint_state_publisher",
        default_value="true"
    )

    # 產生 robot_description 參數，使用 xacro 處理 urdf
    robot_desc = ParameterValue(Command(["xacro ", LaunchConfiguration("urdf_path")]))
    
    # 啟動 robot_state_publisher 節點，發佈機器人狀態
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description":robot_desc}]
        
    )

    # 啟動 joint_state_publisher 節點（可選），用於關節狀態模擬
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        condition=IfCondition(LaunchConfiguration("use_joint_state_publisher"))

    )

    # 回傳 LaunchDescription，包含所有啟動項目
    return LaunchDescription([
        model,
        use_joint_state_publisher,
        robot_state_publisher,
        joint_state_publisher,
    ])