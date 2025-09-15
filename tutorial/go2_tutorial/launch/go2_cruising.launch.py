from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

# 生成啟動描述函數
def generate_launch_description():
    # 獲取 go2_driver 套件的共享目錄
    go2_driver_pkg = get_package_share_directory("go2_driver")

    # 包含 go2_driver 的啟動文件
    go2_driver_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=os.path.join(go2_driver_pkg, "launch", "driver.launch.py")
        )
    )

    # 獲取 go2_tutorial 套件的共享目錄
    go2_tutorial_pkg = get_package_share_directory("go2_tutorial")

    # 包含 go2_ctrl 的啟動文件
    go2_ctrl_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=os.path.join(go2_tutorial_pkg, "launch", "go2_ctrl.launch.py")
        )
    )

    # 定義 go2_cruising_service 節點
    cru_service_node = Node(
        package="go2_tutorial",
        executable="go2_cruising_service",
        parameters=[os.path.join(go2_tutorial_pkg, "params", "go2_cruising_service.yaml")]
    )

    # 返回啟動描述，包含上述所有啟動文件和節點
    return LaunchDescription([
        go2_driver_launch,
        go2_ctrl_launch,
        cru_service_node
    ])