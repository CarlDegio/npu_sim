import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('npu_sim'), 'urdf', 'car.urdf')
    assert os.path.exists(urdf)

    rviz_config_dir = os.path.join(get_package_share_directory('npu_sim'), 'config', 'model.rviz')
    assert os.path.exists(rviz_config_dir)
    print("!!!!",urdf)
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
             arguments=[urdf]
        )
        # Node(
        #     package='turtlesim',
        #     namespace='turtlesim1',
        #     executable='turtlesim_node',
        #     name='sim'
        # ),
        # Node(
        #     package='turtlesim',
        #     namespace='turtlesim2',
        #     executable='turtlesim_node',
        #     name='sim'
        # ),
        # Node(
        #     package='turtlesim',
        #     executable='mimic',
        #     name='mimic',
        #     remappings=[
        #         ('/input/pose', '/turtlesim1/turtle1/pose'),
        #         ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
        #     ]
        # )
    ])
