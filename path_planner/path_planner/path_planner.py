#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
import numpy as np
import heapq
import math
import time

class BalancedDualRobotPathPlanner(Node):
    def __init__(self):
        super().__init__('multi_robot_path_planner')  # 🔄 重命名節點
        
        # 平衡的安全參數
        self.robot_radius = 0.2          # 機器人半徑
        self.safety_margin = 0.15        # 安全邊距
        self.total_safety_distance = self.robot_radius + self.safety_margin
        
        # 搜索參數
        self.max_search_nodes = 30000    # 合理的搜索範圍
        self.diagonal_cost = 1.414       # 對角線移動代價
        self.straight_cost = 1.0         # 直線移動代價
        
        # 路徑質量參數
        self.min_clearance = 0.1         # 最小間隙
        self.preferred_clearance = 0.3   # 偏好間隙
        
        # 🔄 修改：擴展為4台機器人
        self.robot_ids = ['B01', 'B02', 'B03', 'B04']  # 原來是 'robot1', 'robot2'
        
        # 初始化變量 - 擴展為4台機器人
        self.has_map = False
        self.current_paths = {robot_id: None for robot_id in self.robot_ids}
        self.current_goals = {robot_id: None for robot_id in self.robot_ids}
        self.robot_positions = {robot_id: None for robot_id in self.robot_ids}
        self.map_data = None
        self.resolution = None
        self.origin = None
        self.width = None
        self.height = None
        
        # 統計信息 - 擴展為4台機器人
        self.planning_stats = {
            robot_id: {'success': 0, 'failed': 0, 'avg_length': 0} 
            for robot_id in self.robot_ids
        }
        
        # 🔄 修改：為4台機器人創建發布者
        self.path_publishers = {}
        self.viz_path_publishers = {}
        
        for robot_id in self.robot_ids:
            # Float32MultiArray 發布者 (給 path_follow 使用)
            self.path_publishers[robot_id] = self.create_publisher(
                Float32MultiArray, 
                f'/{robot_id}/path',  # 修改topic格式
                10
            )
            
            # Path 發布者 (用於視覺化)
            self.viz_path_publishers[robot_id] = self.create_publisher(
                Path, 
                f'/{robot_id}/planned_path', 
                10
            )
        
        # 地圖訂閱者
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            'merge_map',
            self.map_callback,
            10
        )
        
        # 🔄 修改：為4台機器人創建訂閱者
        for robot_id in self.robot_ids:
            # 里程計訂閱者
            self.create_subscription(
                Odometry,
                f'/{robot_id}/odom',
                lambda msg, rid=robot_id: self.odom_callback(msg, rid),
                10
            )
            
            # 目標點訂閱者
            self.create_subscription(
                PoseStamped,
                f'/{robot_id}/goal',
                lambda msg, rid=robot_id: self.goal_callback(msg, rid),
                10
            )
        
        # 定時器
        self.create_timer(1.5, self.monitor_robots)
        self.create_timer(30.0, self.report_statistics)  # 統計報告定時器
        
        self.get_logger().info(
            f'🤖 Multi-Robot Path Planner started for {len(self.robot_ids)} robots: {self.robot_ids}'
        )
        self.get_logger().info(
            f'🛡️ Advanced Path Planner started - Safety: {self.total_safety_distance:.2f}m, Search limit: {self.max_search_nodes}'
        )

    def map_callback(self, msg):
        """地圖回調"""
        self.resolution = msg.info.resolution
        self.origin = msg.info.origin
        self.width = msg.info.width
        self.height = msg.info.height
        
        # 二值化地圖
        self.map_data = np.array(msg.data).reshape((self.height, self.width))
        self.map_data = np.where(self.map_data > 50, 1, 0)
        
        self.has_map = True
        
        # 減少地圖更新日誌
        if not hasattr(self, 'map_update_count'):
            self.map_update_count = 0
        self.map_update_count += 1
        
        if self.map_update_count % 50 == 0:  # 每50次更新才記錄一次
            self.get_logger().info(f'Map updated (count: {self.map_update_count})')

    def odom_callback(self, msg, robot_id):
        """更新機器人位置"""
        self.robot_positions[robot_id] = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

    def goal_callback(self, msg, robot_id):
        """處理新的目標點 - 改進的兩級策略"""
        if not self.has_map or self.robot_positions[robot_id] is None:
            self.get_logger().warn(f'Cannot plan path for {robot_id}: waiting for map or robot position')
            return
            
        goal = (msg.pose.position.x, msg.pose.position.y)
        self.current_goals[robot_id] = goal
        
        start_time = time.time()
        path = None
        
        # 策略1：標準A*路徑規劃（首選）
        try:
            path = self.plan_safe_path(self.robot_positions[robot_id], goal)
            planning_mode = "safe"
        except Exception as e:
            self.get_logger().debug(f'Safe planning failed for {robot_id}: {e}')
        
        # 策略2：放寬安全檢查的A*（備選）
        if not path:
            try:
                path = self.plan_relaxed_path(self.robot_positions[robot_id], goal)
                planning_mode = "relaxed"
            except Exception as e:
                self.get_logger().debug(f'Relaxed planning failed for {robot_id}: {e}')
                planning_mode = "failed"
        
        # 發布結果
        if path and len(path) >= 2:
            # 輕度路徑平滑
            smoothed_path = self.light_smooth_path(path)
            self.current_paths[robot_id] = smoothed_path
            self.publish_path(smoothed_path, robot_id)
            
            # 更新統計
            self.planning_stats[robot_id]['success'] += 1
            path_length = self.calculate_path_length(smoothed_path)
            stats = self.planning_stats[robot_id]
            stats['avg_length'] = (stats['avg_length'] * (stats['success'] - 1) + path_length) / stats['success']
            
            planning_time = time.time() - start_time
            self.get_logger().info(
                f'Path planned for {robot_id}: {len(smoothed_path)} waypoints, '
                f'{path_length:.2f}m, {planning_time:.3f}s ({planning_mode})'
            )
        else:
            self.planning_stats[robot_id]['failed'] += 1
            self.get_logger().error(
                f'Path planning failed for {robot_id}! '
                f'Start: ({self.robot_positions[robot_id][0]:.2f}, {self.robot_positions[robot_id][1]:.2f}), '
                f'Goal: ({goal[0]:.2f}, {goal[1]:.2f})'
            )

    def plan_safe_path(self, start, goal):
        """標準安全路徑規劃"""
        start_map = self.world_to_map(*start)
        goal_map = self.world_to_map(*goal)
        
        # 邊界檢查
        if not self.is_point_in_bounds(start_map) or not self.is_point_in_bounds(goal_map):
            return None
        
        # 安全性檢查並修正
        safe_start = self.ensure_point_safety(start_map, self.total_safety_distance)
        safe_goal = self.ensure_point_safety(goal_map, self.total_safety_distance)
        
        if not safe_start or not safe_goal:
            return None
        
        # A*搜索
        path_map = self.a_star_with_clearance(safe_start, safe_goal, self.total_safety_distance)
        if not path_map:
            return None
            
        # 轉換回世界坐標
        return [self.map_to_world(p[0], p[1]) for p in path_map]

    def plan_relaxed_path(self, start, goal):
        """放寬安全要求的路徑規劃"""
        start_map = self.world_to_map(*start)
        goal_map = self.world_to_map(*goal)
        
        if not self.is_point_in_bounds(start_map) or not self.is_point_in_bounds(goal_map):
            return None
        
        # 使用最小安全距離
        relaxed_safety = self.min_clearance
        safe_start = self.ensure_point_safety(start_map, relaxed_safety)
        safe_goal = self.ensure_point_safety(goal_map, relaxed_safety)
        
        if not safe_start or not safe_goal:
            return None
        
        # 使用放寬的A*搜索
        path_map = self.a_star_with_clearance(safe_start, safe_goal, relaxed_safety)
        if not path_map:
            return None
            
        return [self.map_to_world(p[0], p[1]) for p in path_map]

    def a_star_with_clearance(self, start, goal, required_clearance):
        """帶間隙偏好的A*搜索"""
        frontier = []
        heapq.heappush(frontier, (0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}
        
        searched_nodes = 0
        
        while frontier and searched_nodes < self.max_search_nodes:
            current_cost, current = heapq.heappop(frontier)
            searched_nodes += 1
            
            if current == goal:
                # 重建路徑
                path = []
                while current is not None:
                    path.append(current)
                    current = came_from[current]
                return path[::-1]
            
            # 探索鄰居
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                neighbor = (current[0] + dr, current[1] + dc)
                
                if not self.is_point_in_bounds(neighbor):
                    continue
                
                if not self.has_required_clearance(neighbor, required_clearance):
                    continue
                
                # 計算移動代價
                move_cost = self.diagonal_cost if self.is_diagonal_move(current, neighbor) else self.straight_cost
                
                # 間隙偏好修正
                clearance_bonus = self.get_clearance_bonus(neighbor)
                new_cost = cost_so_far[current] + move_cost - clearance_bonus
                
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + self.heuristic(neighbor, goal)
                    heapq.heappush(frontier, (priority, neighbor))
                    came_from[neighbor] = current
        
        return None

    def has_required_clearance(self, pos, clearance_distance):
        """檢查點是否有所需的間隙"""
        row, col = pos
        
        if not self.is_point_in_bounds(pos):
            return False
        
        if self.map_data[row, col] != 0:
            return False
        
        # 檢查周圍的間隙
        clearance_cells = int(clearance_distance / self.resolution) + 1
        
        for dr in range(-clearance_cells, clearance_cells + 1):
            for dc in range(-clearance_cells, clearance_cells + 1):
                check_pos = (row + dr, col + dc)
                if self.is_point_in_bounds(check_pos):
                    if self.map_data[check_pos[0], check_pos[1]] != 0:
                        distance = math.sqrt(dr*dr + dc*dc) * self.resolution
                        if distance <= clearance_distance:
                            return False
        
        return True

    def get_clearance_bonus(self, pos):
        """獲得間隙獎勵"""
        if self.has_required_clearance(pos, self.preferred_clearance):
            return 0.2
        elif self.has_required_clearance(pos, self.min_clearance + 0.1):
            return 0.1
        return 0

    def ensure_point_safety(self, point, required_clearance):
        """確保點的安全性"""
        if self.has_required_clearance(point, required_clearance):
            return point
        
        # 尋找附近的安全點
        return self.find_nearest_safe_point(point, required_clearance)

    def find_nearest_safe_point(self, point, required_clearance, max_search_radius=15):
        """找到附近的安全點"""
        row, col = point
        
        for radius in range(1, max_search_radius + 1):
            # 優先檢查4個主要方向
            for dr, dc in [(0, radius), (0, -radius), (radius, 0), (-radius, 0)]:
                new_point = (row + dr, col + dc)
                if self.has_required_clearance(new_point, required_clearance):
                    return new_point
            
            # 然後檢查對角線方向
            for dr, dc in [(radius, radius), (radius, -radius), (-radius, radius), (-radius, -radius)]:
                new_point = (row + dr, col + dc)
                if self.has_required_clearance(new_point, required_clearance):
                    return new_point
        
        return None

    def light_smooth_path(self, path):
        """輕度路徑平滑，保持安全性"""
        if len(path) <= 2:
            return path
            
        smoothed = [path[0]]
        i = 0
        
        while i < len(path) - 1:
            # 嘗試跳過最多3個中間點
            max_skip = min(4, len(path) - i - 1)
            
            for skip in range(max_skip, 0, -1):
                j = i + skip
                if j < len(path) and self.is_path_segment_safe(path[i], path[j]):
                    smoothed.append(path[j])
                    i = j
                    break
            else:
                # 如果無法跳過，移動到下一個點
                i += 1
                if i < len(path):
                    smoothed.append(path[i])
        
        return smoothed

    def is_path_segment_safe(self, start, end):
        """檢查路徑段是否安全"""
        start_map = self.world_to_map(*start)
        end_map = self.world_to_map(*end)
        
        points = self.bresenham_line(start_map[0], start_map[1], end_map[0], end_map[1])
        
        # 檢查路徑上的每個點
        for point in points[::2]:  # 每隔一個點檢查
            if not self.has_required_clearance(point, self.min_clearance):
                return False
        
        return True

    def is_diagonal_move(self, pos1, pos2):
        """檢查是否為對角線移動"""
        return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1]) == 2

    def calculate_path_length(self, path):
        """計算路徑長度"""
        if len(path) < 2:
            return 0.0
        
        total_length = 0.0
        for i in range(1, len(path)):
            dx = path[i][0] - path[i-1][0]
            dy = path[i][1] - path[i-1][1]
            total_length += math.sqrt(dx*dx + dy*dy)
        
        return total_length

    def is_point_in_bounds(self, pos):
        """檢查點是否在邊界內"""
        row, col = pos
        return 0 <= row < self.height and 0 <= col < self.width

    def world_to_map(self, x, y):
        """世界坐標轉地圖坐標"""
        mx = int((x - self.origin.position.x) / self.resolution)
        my = int((y - self.origin.position.y) / self.resolution)
        return (my, mx)

    def map_to_world(self, row, col):
        """地圖坐標轉世界坐標"""
        x = col * self.resolution + self.origin.position.x
        y = row * self.resolution + self.origin.position.y
        return (x, y)

    def heuristic(self, a, b):
        """啟發式函數（歐幾里得距離）"""
        return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

    def bresenham_line(self, x0, y0, x1, y1):
        """Bresenham直線算法"""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = 1 if x1 > x0 else -1
        sy = 1 if y1 > y0 else -1
        
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                points.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                points.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
                
        points.append((x, y))
        return points

    def monitor_robots(self):
        """監控機器人狀態 - 修改為支援4台機器人"""
        for robot_id in self.robot_ids:  # 🔄 修改這裡
            if (self.current_paths[robot_id] and 
                self.current_goals[robot_id] and 
                self.robot_positions[robot_id]):
                
                robot_pos = self.robot_positions[robot_id]
                goal_pos = self.current_goals[robot_id]
                
                # 檢查是否接近目標
                dist_to_goal = math.sqrt(
                    (robot_pos[0] - goal_pos[0])**2 + 
                    (robot_pos[1] - goal_pos[1])**2
                )
                
                if dist_to_goal < 0.4:  # 接近目標
                    self.current_paths[robot_id] = None
                    self.current_goals[robot_id] = None

    def report_statistics(self):
        """報告統計信息 - 修改為支援4台機器人"""
        for robot_id in self.robot_ids:  # 🔄 修改這裡
            stats = self.planning_stats[robot_id]
            total = stats['success'] + stats['failed']
            if total > 0:
                success_rate = stats['success'] / total
                avg_length = stats['avg_length']
                self.get_logger().info(
                    f'{robot_id}: {stats["success"]}/{total} success '
                    f'({success_rate:.1%}), avg length: {avg_length:.2f}m'
                )

    def publish_path(self, path, robot_id):
        """發布路徑"""
        # 發布 Float32MultiArray
        float_msg = Float32MultiArray()
        float_msg.data = [coord for point in path for coord in point]
        self.path_publishers[robot_id].publish(float_msg)
        
        # 發布 Path 用於視覺化
        path_msg = Path()
        path_msg.header.frame_id = 'merge_map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for x, y in path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
            
        self.viz_path_publishers[robot_id].publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BalancedDualRobotPathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
