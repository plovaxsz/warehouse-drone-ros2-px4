#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class ScanToCloud(Node):
    def __init__(self):
        super().__init__('scan_to_cloud')
        
        # --- FIX CRASH ---
        try:
            self.declare_parameter('use_sim_time', True)
        except:
            pass
        
        # --- KAPASITAS BESAR (100) ---
        qos = QoSProfile(
            depth=100, 
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.listener_callback, qos)
            
        self.publisher = self.create_publisher(PointCloud2, '/scan_cloud', 100)
        self.lp = LaserProjection()

    def listener_callback(self, msg):
        try:
            cloud = self.lp.projectLaser(msg)
            self.publisher.publish(cloud)
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = ScanToCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
