from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os



def generate_launch_description():

    go2_driver_pkg = get_package_share_directory("go2_driver_py")
    go2_driver_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=os.path.join(go2_driver_pkg, "launch", "driver.launch.py")
        )
    )

    go2_tutorial_pkg = get_package_share_directory("go2_tutorial_py")
    go2_ctrl_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=os.path.join(go2_tutorial_pkg, "launch", "go2_ctrl.launch.py")
        )
    )

    nav_server_node = Node(
        package="go2_tutorial_py",
        executable="go2_nav_server",
        parameters=[os.path.join(go2_tutorial_pkg, "params", "go2_nav.yaml")]
    )


    return LaunchDescription([
        go2_driver_launch,
        go2_ctrl_launch,
        nav_server_node,


    ])