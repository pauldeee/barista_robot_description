import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import TimerAction


def generate_launch_description():

    package_description = "barista_robot_description"
    xacro_file_path = os.path.join(get_package_share_directory(
        package_description), "xacro", "barista_robot_model.urdf.xacro")

    # use Command to call xacro and convert the .xacro file to .urdf format.
    robot_desc_content = Command(['xacro ', xacro_file_path])

    # gazebo
    gazebo_ros_package_dir = get_package_share_directory('gazebo_ros')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_ros_package_dir, '/launch/gazebo.launch.py']))

    # robot state pub
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty=True,
        parameters=[{'use_sim_time': True,
                     'robot_description': robot_desc_content}],
        output="screen"
    )

    # joint state publisher
    joint_state_pub = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher'
    )

    # rviz conf
    rviz_config_dir = os.path.join(get_package_share_directory(
        package_description), 'rviz', 'rviz.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir])

    # spawn
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity', 'barista_robot', '-topic', '/robot_description']
    )

    # delay spawn into gazebo
    delay_spawn = TimerAction(period=10.0, actions=[spawn_robot])

    # create and return launch description object
    return LaunchDescription(
        [
            gazebo_launch,
            robot_state_publisher_node,
            joint_state_pub,
            delay_spawn,
            rviz_node
        ]
    )
