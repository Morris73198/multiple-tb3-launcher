#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """啟動多機器人導航系統"""
    
    # 機器人列表
    robot_namespaces = ['B01', 'B02', 'B03', 'B04']
    
    # 啟動描述
    launch_descriptions = []
    
    # 1. 啟動路徑規劃器 (單一節點處理所有機器人)
    path_planner_node = Node(
        package='multi_robot_exploration',
        executable='path_planner',
        name='multi_robot_path_planner',
        output='screen',
        parameters=[
            {'frame_id': 'merge_map'}
        ]
    )
    launch_descriptions.append(path_planner_node)
    
    # 2. 為每個機器人啟動路徑跟隨器
    for robot_ns in robot_namespaces:
        path_follower_node = Node(
            package='path_follow',
            executable='path_follow',
            name=f'path_follower_{robot_ns}',
            output='screen',
            parameters=[
                {'robot_namespace': robot_ns}
            ]
        )
        launch_descriptions.append(path_follower_node)
    
    # 3. 啟動命令調度器
    command_dispatcher_node = Node(
        package='multi_robot_exploration',
        executable='robot_command_dispatcher',
        name='robot_command_dispatcher',
        output='screen'
    )
    launch_descriptions.append(command_dispatcher_node)
    
    return LaunchDescription(launch_descriptions)