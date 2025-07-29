

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    # world_file_name = 'empty_worlds/' + 'waffle_pi_multi.model'
    world_file_name = 'a'

    world = os.path.join(get_package_share_directory('turtlebot3_gazebo'),
                         'worlds', world_file_name)
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    # static publisher between /map and /B01/odom
    # static publisher between /map and /B02/odom
    # static publisher between /map and /B03/odom
    # static publisher between /map and /B04/odom

    static_tf_node_B01 = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['1', '1', '0', '0', '0', '0', 'map', 'B01/odom'],
            output='screen'
        )

    static_tf_node_B02 = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['-1', '1', '0', '0', '0', '0', 'map', 'B02/odom'],
            output='screen'
        )
    
    static_tf_node_B03 = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['-1', '-1', '0', '0', '0', '0', 'map', 'B03/odom'],
            output='screen'
        )
    
    static_tf_node_B04 = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['1', '-1', '0', '0', '0', '0', 'map', 'B04/odom'],
            output='screen'
        )
    

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),

        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', use_sim_time],
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher_m.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time, 'robot_namespace': 'B01', 'frame_id' : 'B01/'}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher_m.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time, 'robot_namespace': 'B02', 'frame_id' : 'B02/'}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher_m.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time, 'robot_namespace': 'B03' , 'frame_id' : 'B03/'}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher_m.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time, 'robot_namespace': 'B04', 'frame_id' : 'B04/'}.items(),
        ),
        static_tf_node_B01,
        static_tf_node_B02,
        static_tf_node_B03,
        static_tf_node_B04,
    ])

