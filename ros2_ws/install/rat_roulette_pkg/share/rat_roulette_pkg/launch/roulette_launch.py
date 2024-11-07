from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    depthai_launch_file = os.path.join(
        get_package_share_directory('depthai_examples'),
        'launch',
        'mobile_publisher.launch.py'
    )
    depthai_cmd = IncludeLaunchDescription(
        depthai_launch_file,
        launch_arguments={}.items()
    )    
    robot_control_cmd = Node(
        package='rat_roulette_pkg',
        executable='rat_roulette_node',
        name='rat_roulette',
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(depthai_cmd)
    ld.add_action(robot_control_cmd)

    return ld
