from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
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

    # 定義 go2_state 節點
    state_node = Node(
        package="go2_tutorial",
        executable="go2_state",  # 節點執行檔名稱
        parameters=os.path.join(get_package_share_directory("go2_tutorial"), "params", "go2_state.yaml")
    )

    # 返回啟動描述，包含上述所有啟動文件和節點
    return LaunchDescription([
        go2_driver_launch,
        state_node,
    ])