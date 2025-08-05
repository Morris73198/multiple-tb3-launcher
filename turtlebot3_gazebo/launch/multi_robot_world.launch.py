import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    
    # 使用空的world文件，而不是預先放入機器人
    world_file_name = 'empty_world.world'  # 或者你想要的其他世界文件

    world = os.path.join(get_package_share_directory('turtlebot3_gazebo'),
                         'worlds', world_file_name)
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # 機器人SDF文件路徑
    robot_sdf = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        'turtlebot3_waffle_pi',
        'model.sdf'
    )
    
    # RViz 配置文件路徑
    rviz_config_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'rviz')
    rviz_config_file = os.path.join(rviz_config_dir, 'tb3_gazebo.rviz')
    
    # 機器人位置定義
    robots = [
        {"name": "robot1", "x_pose": 1.0, "y_pose": 1.0, "z_pose": 0.01},
        {"name": "robot2", "x_pose": -1.0, "y_pose": 1.0, "z_pose": 0.01},
        {"name": "robot3", "x_pose": -1.0, "y_pose": -1.0, "z_pose": 0.01},
        {"name": "robot4", "x_pose": 1.0, "y_pose": -1.0, "z_pose": 0.01},
    ]
    
    # static publisher between /map and robot odom frames
    # 重新添加静态TF，连接map到各个机器人的odom frames
    static_tf_nodes = []
    for i, robot in enumerate(robots):
        static_tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                str(robot["x_pose"]), str(robot["y_pose"]), '0', '0', '0', '0', 
                'map', f'{robot["name"]}/odom'
            ],
            output='screen'
        )
        static_tf_nodes.append(static_tf_node)
    
    # 動態spawn機器人節點
    spawn_robots_cmds = []
    for robot in robots:
        spawn_robot_cmd = Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            output="screen",
            arguments=[
                "-entity", robot["name"],
                "-file", robot_sdf,
                "-robot_namespace", robot["name"],  # 關鍵：設定正確的namespace
                "-x", TextSubstitution(text=str(robot["x_pose"])),
                "-y", TextSubstitution(text=str(robot["y_pose"])),
                "-z", TextSubstitution(text=str(robot["z_pose"])),
                "-R", "0.0",
                "-P", "0.0",
                "-Y", "0.0",
            ],
        )
        spawn_robots_cmds.append(spawn_robot_cmd)
    
    # 為每個機器人創建robot_state_publisher
    robot_state_publishers = []
    for robot in robots:
        robot_state_pub = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'namespace': robot["name"],
                'use_namespace': 'True',
            }.items(),
        )
        robot_state_publishers.append(robot_state_pub)
    
    # 為每個機器人創建獨立的 RViz 節點
    rviz_nodes = []
    for i, robot in enumerate(robots):
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name=f'rviz2_{robot["name"]}',
            namespace=robot["name"],
            arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
            parameters=[
                {'use_sim_time': use_sim_time},
            ],
            output='screen',
            remappings=[
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static'),
                ('/scan', f'/{robot["name"]}/scan'),
                ('/odom', f'/{robot["name"]}/odom'),
                ('/cmd_vel', f'/{robot["name"]}/cmd_vel'),
            ]
        )
        rviz_nodes.append(rviz_node)
    
    # 簡單地圖發布器（用於可視化）
    map_publisher = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '/map', 'nav_msgs/msg/OccupancyGrid',
            '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, info: {resolution: 0.05, width: 400, height: 400, origin: {position: {x: -10.0, y: -10.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, data: []}',
            '-r', '1'
        ],
        output='screen'
    )

    # 組裝啟動描述
    launch_actions = [
        # 啟動 Gazebo
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
    ]
    
    # 添加靜態TF發布器
    launch_actions.extend(static_tf_nodes)
    
    # 延遲spawn機器人（等待Gazebo完全啟動）
    for i, spawn_cmd in enumerate(spawn_robots_cmds):
        launch_actions.append(
            TimerAction(
                period=3.0 + i * 1.0,  # 錯開spawn時間
                actions=[spawn_cmd]
            )
        )
    
    # 延遲啟動robot_state_publishers
    for i, rsp in enumerate(robot_state_publishers):
        launch_actions.append(
            TimerAction(
                period=5.0 + i * 0.5,
                actions=[rsp]
            )
        )
    
    # 延遲啟動地圖發布器
    launch_actions.append(
        TimerAction(
            period=6.0,
            actions=[map_publisher]
        )
    )
    
    # 延遲啟動RViz窗口
    for i, rviz_node in enumerate(rviz_nodes):
        launch_actions.append(
            TimerAction(
                period=8.0 + i * 1.0,  # 錯開RViz啟動時間
                actions=[rviz_node]
            )
        )

    return LaunchDescription(launch_actions)