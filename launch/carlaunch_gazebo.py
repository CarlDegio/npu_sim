import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    world_dir = os.path.join(get_package_share_directory('npu_sim'), 'world', 'default.world')
    assert os.path.exists(world_dir)  # no exist
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'world': world_dir}.items(),
    )

    xacro_file = os.path.join(get_package_share_directory('npu_sim'), 'urdf', 'car.xacro')
    assert os.path.exists(xacro_file)

    doc=xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description_config=doc.toxml()
    robot_description = {'robot_description': robot_description_config}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    rviz_config_dir = os.path.join(get_package_share_directory('npu_sim'), 'config', 'world.rviz')
    assert os.path.exists(rviz_config_dir)

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'arm_car'],
                        output='screen')

    load_joint_state_broadcaster=ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start','joint_state_broadcaster'],
        output='screen'
    )

    load_joint_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'arm_joint_controller'],
        output='screen'
    )

    load_camera_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'camera_joint_controller'],
        output='screen'
    )

    rviz2=Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen')

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_controller,
                on_exit=[load_camera_controller],
            )
        ),
        gazebo,
        rviz2,
        node_robot_state_publisher,
        spawn_entity
    ])
