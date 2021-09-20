import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    xacro_file = os.path.join(get_package_share_directory('npu_sim'), 'urdf', 'car.xacro')
    assert os.path.exists(xacro_file), "The box_bot.xacro doesnt exist in " + str(xacro_file)
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    rviz_config_dir = os.path.join(get_package_share_directory('npu_sim'), 'config', 'model.rviz')
    assert os.path.exists(rviz_config_dir)
    #assert os.path.exists(urdf)  # no exist

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output = 'screen'
        ),
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             name='robot_state_publisher',
             output='screen',
             parameters=[{"robot_description": robot_desc}],
        )
    ])
