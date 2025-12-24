#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class ScanToCloud(Node):
    def __init__(self):
        super().__init__('scan_to_cloud')
        try:
            if not self.has_parameter('use_sim_time'):
                self.declare_parameter('use_sim_time', True)
        except:
            pass
        
        qos = QoSProfile(
            depth=10, 
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.listener_callback, qos)
            
        self.publisher = self.create_publisher(PointCloud2, '/scan_cloud', 10)
        self.lp = LaserProjection()
        self.process_count = 0

    def listener_callback(self, msg):
        self.process_count += 1
        if self.process_count % 3 != 0: # Throttle: Proses 1 dari 3 data
            return

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
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
