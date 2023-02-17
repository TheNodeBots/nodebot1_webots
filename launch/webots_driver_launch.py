import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
#from webots_ros2_driver.webots_launcher import Ros2SupervisorLauncher

def generate_launch_description():
    package_dir = get_package_share_directory('nodebot1_webots')
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'webots_chassis.urdf')).read_text()

    # fetch controller URL from environment
    webots_controller_url = os.environ.get('WEBOTS_CONTROLLER_URL') + 'NodeBot1'

    webots_chassis_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL': webots_controller_url},
        parameters=[
            {'robot_description': robot_description},
        ]
    )

    return LaunchDescription([
        webots_chassis_driver
    ])
