#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class PX4TFBroadcaster(Node):
    def __init__(self):
        super().__init__('px4_tf_broadcaster')
        
        # PENTING: Gunakan Waktu Simulasi
        self.declare_parameter('use_sim_time', True)
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odom_callback,
            qos_profile)
            
        self.tf_broadcaster = TransformBroadcaster(self)

    def odom_callback(self, msg):
        t = TransformStamped()
        # PENTING: Ambil waktu dari Node (yang sinkron dengan Gazebo)
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        # Mapping Koordinat (NED ke ENU)
        t.transform.translation.x = float(msg.position[0])
        t.transform.translation.y = float(-msg.position[1])
        t.transform.translation.z = float(-msg.position[2])
        t.transform.rotation.x = float(msg.q[1])
        t.transform.rotation.y = float(-msg.q[2])
        t.transform.rotation.z = float(-msg.q[3])
        t.transform.rotation.w = float(msg.q[0])

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = PX4TFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
