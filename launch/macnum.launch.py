import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    # use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # urdf = os.path.join(get_package_share_directory('box_car_description'), 'robot/', 'box_bot.urdf')    
    # assert os.path.exists(urdf), "Thebox_bot.urdf doesnt exist in "+str(urdf)

    xacro_file = os.path.join(get_package_share_directory('npu_sim'), 'urdf', 'four_macnum.urdf.xacro')

    assert os.path.exists(xacro_file), "The box_bot.xacro doesnt exist in "+str(xacro_file)
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    #print(robot_desc)

    rviz_config_dir = os.path.join(get_package_share_directory('npu_sim'), 'config', 'model.rviz')
    assert os.path.exists(rviz_config_dir)
    # with open(urdf, 'r') as infp:
    #     robot_desc = infp.read()

    return LaunchDescription([
        # DeclareLaunchArgument(
        #     'use_sim_time',
        #     default_value='false',
        #     description='Use simulation (Gazebo) clock if true'),
        # Node(
        #     package='box_car_description',
        #     executable='spawn_box_bot.py',
        #     arguments=[robot_desc],
        #     output='screen'
        # ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_desc}],
            output="screen"),
        # No mesh,couldn't use yet
        # https://github.com/MapleHan/four_macnum_car/tree/master/four_macnum_description/meshes
        # maybe fix it
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'
        ),
    ])
