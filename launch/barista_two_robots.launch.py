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

    
    # gazebo
    gazebo_ros_package_dir = get_package_share_directory('gazebo_ros')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_ros_package_dir, '/launch/gazebo.launch.py']))


    # sim time?
    use_sim_time = True

    # names
    robot_name_1 = "rick"
    robot_name_2 = "morty"

    # use Command to call xacro and convert the .xacro file to .urdf format.
    robot_1_desc_content = Command([
    'xacro ', xacro_file_path,
    ' robot_name:=', robot_name_1
    ])
    robot_2_desc_content = Command([
    'xacro ', xacro_file_path,
    ' robot_name:=', robot_name_2,
    ' tray_color_gazebo:=Red',
    ' tray_color_rviz:=red',
    ])


     # rviz conf
    rviz_config_dir = os.path.join(get_package_share_directory(
        package_description), 'rviz', 'two_robots.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config_dir])



    # robot state pub
    rsp_robot_1 = Node(package='robot_state_publisher',
        executable='robot_state_publisher',
        name=robot_name_1+'_robot_state_publisher_node',
        namespace=robot_name_1,
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_1_desc_content}],
        output="screen"
        )

    rsp_robot_2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=robot_name_2+'_robot_state_publisher_node',
        namespace=robot_name_2,
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_2_desc_content}],
        output="screen"
    )

    # spawn
    spawn_robot_1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity', 'barista_robot_1', '-x', '0.0', '-y', '0.0', '-z', '0.0',
         '-topic', robot_name_1+'/robot_description']
    )

    spawn_robot_2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity', 'barista_robot_2', '-x', '1.0', '-y', '1.0', '-z', '0.0',
         '-topic', robot_name_2+'/robot_description']
    )

    # delay spawn into gazebo
    delay_spawn = TimerAction(period=10.0, actions=[spawn_robot_1, spawn_robot_2])

    static_pub_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=robot_name_1+'_static_tf_publisher',
        output='screen',
        parameters=[],
        remappings=[],
        # x y z yaw pitch roll frame_id child_frame_id
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', robot_name_1+'_odom']
        )

    static_pub_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=robot_name_2+'_static_tf_publisher',
        output='screen',
        parameters=[],
        remappings=[],
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', robot_name_2+'_odom']
        )

   
    # create and return launch description object
    return LaunchDescription(
        [
            gazebo_launch,
            rviz_node,
            rsp_robot_1,
            rsp_robot_2,
            static_pub_1,
            static_pub_2,
            delay_spawn
        ]
    )
