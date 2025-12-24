#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
import math
import time

class AutoExplorer(Node):
    def __init__(self):
        super().__init__('auto_explorer')
        
        # Subscribe ke Map
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
            
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos)
            
        # Action Client untuk Navigasi
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.latest_map = None
        self.is_navigating = False
        self.get_logger().info("Auto Explorer Started! Waiting for Map...")

    def map_callback(self, msg):
        self.latest_map = msg
        if not self.is_navigating:
            self.decide_next_goal()

    def get_frontiers(self, grid, width, height, resolution, origin_x, origin_y):
        # 0 = Free, 100 = Occupied, -1 = Unknown
        # Frontier adalah cell '0' yang bertetangga dengan '-1'
        
        frontiers = []
        
        # Kita scan grid dengan langkah kasar untuk performa (skip 5 pixel)
        step = 5 
        
        for y in range(1, height - 1, step):
            for x in range(1, width - 1, step):
                idx = x + y * width
                
                # Jika cell ini FREE (0)
                if grid[idx] == 0:
                    # Cek 4 tetangga, apakah ada yang UNKNOWN (-1)?
                    neighbors = [
                        grid[x+1 + y*width],     # Kanan
                        grid[x-1 + y*width],     # Kiri
                        grid[x + (y+1)*width],   # Atas
                        grid[x + (y-1)*width]    # Bawah
                    ]
                    
                    if -1 in neighbors:
                        # Ini adalah Frontier! Simpan koordinat dunia nyatanya
                        real_x = (x * resolution) + origin_x
                        real_y = (y * resolution) + origin_y
                        frontiers.append((real_x, real_y))
        
        return frontiers

    def decide_next_goal(self):
        if self.latest_map is None:
            return

        grid = self.latest_map.data
        width = self.latest_map.info.width
        height = self.latest_map.info.height
        res = self.latest_map.info.resolution
        orig_x = self.latest_map.info.origin.position.x
        orig_y = self.latest_map.info.origin.position.y

        self.get_logger().info(f"Processing Map: {width}x{height}...")
        
        frontiers = self.get_frontiers(grid, width, height, res, orig_x, orig_y)
        
        if not frontiers:
            self.get_logger().info("No frontiers found. Exploration complete or Map empty.")
            return

        # Pilih frontier
        # Strategi: Pilih yang tidak terlalu jauh (agar tracing rak), tapi tidak terlalu dekat (biar gerak)
        # Kita ambil frontier tengah dari list (random heuristic)
        target = frontiers[len(frontiers) // 2]
        
        self.send_goal(target[0], target[1])

    def send_goal(self, x, y):
        self.is_navigating = True
        self.get_logger().info(f"Sending Drone to Frontier: X={x:.2f}, Y={y:.2f}")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.w = 1.0 # Hadap depan standar

        self._action_client.wait_for_server()
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.is_navigating = False
            return

        self.get_logger().info('Goal accepted! Flying...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.is_navigating = False
        self.get_logger().info('Arrived at Frontier! Scanning...')
        # Beri jeda sedikit sebelum cari target baru
        time.sleep(1.0)
        # Panggil ulang logika cari frontier
        self.decide_next_goal()

def main(args=None):
    rclpy.init(args=args)
    node = AutoExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
