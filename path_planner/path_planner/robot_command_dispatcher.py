#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import threading
import sys

class RobotCommandDispatcher(Node):
    def __init__(self):
        super().__init__('robot_command_dispatcher')
        
        # 機器人映射表 (編號 -> 命名空間)
        self.robot_mapping = {
            0: 'B01',
            1: 'B02', 
            2: 'B03',
            3: 'B04'
        }
        
        # 為每個機器人創建目標點發布器
        self.goal_publishers = {}
        for robot_num, robot_ns in self.robot_mapping.items():
            topic_name = f'/{robot_ns}/goal'
            self.goal_publishers[robot_num] = self.create_publisher(
                PoseStamped, 
                topic_name, 
                10
            )
            self.get_logger().info(f'Created goal publisher for robot {robot_num} -> {topic_name}')
        
        self.get_logger().info('🤖 Robot Command Dispatcher started!')
        self.get_logger().info('📝 Usage: <robot_id> <x> <y>')
        self.get_logger().info('📝 Example: 0 6 8  (Robot 0 go to position 6,8)')
        self.get_logger().info('📝 Available robots: 0(B01), 1(B02), 2(B03), 3(B04)')
        
        # 開始命令輸入線程
        self.input_thread = threading.Thread(target=self.command_input_loop, daemon=True)
        self.input_thread.start()

    def send_goal_to_robot(self, robot_id, x, y):
        """發送目標點給指定機器人"""
        if robot_id not in self.robot_mapping:
            self.get_logger().error(f'❌ Invalid robot ID: {robot_id}. Valid IDs: {list(self.robot_mapping.keys())}')
            return False
            
        # 創建目標點消息
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'merge_map'  # 根據你的座標系調整
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.position.x = float(x)
        goal_msg.pose.position.y = float(y)
        goal_msg.pose.position.z = 0.0
        
        # 設置方向 (預設朝向)
        goal_msg.pose.orientation.w = 1.0
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = 0.0
        
        # 發布目標點
        self.goal_publishers[robot_id].publish(goal_msg)
        
        robot_ns = self.robot_mapping[robot_id]
        self.get_logger().info(f'🎯 Robot {robot_id}({robot_ns}) -> Goal: ({x}, {y})')
        return True

    def parse_command(self, command_str):
        """解析命令字符串"""
        try:
            parts = command_str.strip().split()
            if len(parts) != 3:
                return None, None, None, "Format: <robot_id> <x> <y>"
                
            robot_id = int(parts[0])
            x = float(parts[1]) 
            y = float(parts[2])
            
            return robot_id, x, y, None
            
        except ValueError as e:
            return None, None, None, f"Invalid number format: {e}"
        except Exception as e:
            return None, None, None, f"Parse error: {e}"

    def command_input_loop(self):
        """命令輸入循環"""
        self.get_logger().info('💬 Ready for commands! Type commands or "quit" to exit')
        
        while rclpy.ok():
            try:
                # 讀取用戶輸入
                command = input('🤖 Enter command: ').strip()
                
                if command.lower() in ['quit', 'exit', 'q']:
                    self.get_logger().info('👋 Exiting...')
                    break
                    
                if command.lower() in ['help', 'h']:
                    self.show_help()
                    continue
                    
                if not command:
                    continue
                    
                # 解析並執行命令
                robot_id, x, y, error = self.parse_command(command)
                
                if error:
                    self.get_logger().error(f'❌ {error}')
                    continue
                    
                # 發送目標點
                self.send_goal_to_robot(robot_id, x, y)
                
            except KeyboardInterrupt:
                self.get_logger().info('👋 Interrupted, exiting...')
                break
            except EOFError:
                self.get_logger().info('👋 EOF received, exiting...')
                break
            except Exception as e:
                self.get_logger().error(f'❌ Command error: {e}')

    def show_help(self):
        """顯示幫助信息"""
        print('\n📖 Command Help:')
        print('  Format: <robot_id> <x> <y>')
        print('  Example: 0 6 8    (Robot 0 go to position 6,8)')
        print('  Example: 1 -3 2   (Robot 1 go to position -3,2)')
        print('  Example: 2 0 0    (Robot 2 go to origin)')
        print('\n🤖 Available Robots:')
        for robot_num, robot_ns in self.robot_mapping.items():
            print(f'  {robot_num} -> {robot_ns}')
        print('\n⌨️  Commands:')
        print('  help, h  - Show this help')
        print('  quit, q  - Exit program')
        print()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        dispatcher = RobotCommandDispatcher()
        rclpy.spin(dispatcher)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()