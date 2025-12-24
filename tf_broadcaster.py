#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class TFBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')
        self.declare_parameter('use_sim_time', True)
        
        # --- PERBAIKAN: PERBESAR KAPASITAS QOS ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=50  # Gedein dari 1 jadi 50!
        )

        self.subscription = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.listener_callback,
            qos_profile)
            
        self.tf_broadcaster = TransformBroadcaster(self)

    def listener_callback(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = float(msg.position[1])
        t.transform.translation.y = float(msg.position[0])
        t.transform.translation.z = -float(msg.position[2])
        t.transform.rotation.x = msg.q[1]
        t.transform.rotation.y = msg.q[0]
        t.transform.rotation.z = -msg.q[2]
        t.transform.rotation.w = -msg.q[3] 
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = TFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
