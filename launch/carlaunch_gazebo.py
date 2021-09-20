import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('npu_sim'), 'urdf', 'car_gazebo.urdf')
    assert os.path.exists(urdf)

    rviz_config_dir = os.path.join(get_package_share_directory('npu_sim'), 'config', 'world.rviz')
    assert os.path.exists(rviz_config_dir)

    world_dir = os.path.join(get_package_share_directory('npu_sim'), 'world', 'default.world')
    assert os.path.exists(world_dir)  # no exist

    xml = open(urdf, 'r').read()
    xml = xml.replace('"', '\\"')
    spwan_args = '{name: \"arm_car\", xml: \"' + xml + '\" }'

    gazebo = ExecuteProcess(cmd=['gazebo', '--verbose', world_dir,
                                 '-s', 'libgazebo_ros_factory.so'], output='screen')

    spawn_entity=ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', spwan_args],
            output='screen')

    rviz2=Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen')

    # node_robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     arguments=[urdf])

    # load_joint_state_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
    #          'joint_state_broadcaster'],
    #     output='screen'
    # )
    #
    # load_joint_trajectory_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
    #          'joint_trajectory_controller'],
    #     output='screen'
    # )

    return LaunchDescription([
        gazebo,
        spawn_entity,
        rviz2,
        # node_robot_state_publisher,
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
