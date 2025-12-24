#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from px4_msgs.msg import TrajectorySetpoint, OffboardControlMode, VehicleStatus, VehicleOdometry
import numpy as np
import time

class CmdSender(Node):
    def __init__(self):
        super().__init__('cmd_sender')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_callback, qos_profile)
        self.odom_sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, qos_profile)

        # Publishers
        self.traj_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)

        # Variables
        self.nav_vel = [0.0, 0.0, 0.0] # North, East, Yaw
        self.vehicle_yaw = 0.0
        self.current_pos = [0.0, 0.0, 0.0]
        self.hold_pos = [float('nan'), float('nan'), float('nan')] # Posisi kunci saat diam
        self.last_cmd_time = self.get_clock().now()
        self.nav_state = 0
        
        # Timer (30Hz agar lebih cepat dari kebutuhan minimum PX4)
        self.timer = self.create_timer(0.033, self.cmdloop_callback)
        self.get_logger().info("✅ CmdSender Stable Version Active!")

    def status_callback(self, msg):
        self.nav_state = msg.nav_state

    def odom_callback(self, msg):
        self.current_pos = [msg.position[0], msg.position[1], msg.position[2]]
        # Quaternion to Yaw
        q = msg.q
        siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
        self.vehicle_yaw = np.arctan2(siny_cosp, cosy_cosp)

    def cmd_vel_callback(self, msg):
        # Convert Body Frame (ROS) to NED Frame (PX4)
        # cmd_vel.linear.x -> Maju (Body Front)
        # cmd_vel.linear.y -> Geser (Body Right)
        self.nav_vel = [msg.linear.x, -msg.linear.y, -msg.angular.z] # Note: -y karena PX4 y-axis ke kanan
        self.last_cmd_time = self.get_clock().now()
        self.hold_pos = [float('nan'), float('nan'), float('nan')] # Reset hold pos saat ada command baru

    def cmdloop_callback(self):
        # 1. PUBLISH HEARTBEAT (Wajib agar tidak Failsafe)
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        offboard_msg.position = False
        offboard_msg.velocity = True
        offboard_msg.acceleration = False
        self.offboard_mode_pub.publish(offboard_msg)

        # 2. Cek Timeout Command (Jika Nav2 diam > 0.5 detik, aktifkan Position Hold)
        time_diff = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        
        traj_msg = TrajectorySetpoint()
        traj_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        if time_diff > 0.5:
            # --- LOGIKA POSITION HOLD ---
            if np.isnan(self.hold_pos[0]):
                self.hold_pos = list(self.current_pos) # Kunci posisi saat ini
                self.get_logger().info("🛑 Holding Position (No Nav2 Command)", throttle_duration_sec=5)

            # Kirim Velocity 0, tapi PX4 di mode Velocity akan drift.
            # Trik: Kirim velocity 0 tapi dalam frame world (NED)
            traj_msg.velocity = [0.0, 0.0, 0.0]
            traj_msg.yaw = float('nan') # Pertahankan yaw
        else:
            # --- LOGIKA GERAK (VELOCITY CONTROL) ---
            # Transformasi dari Body Frame (relatif terhadap kepala drone) ke NED Frame (World)
            cos_yaw = np.cos(self.vehicle_yaw)
            sin_yaw = np.sin(self.vehicle_yaw)

            # v_north = v_forward * cos(yaw) - v_right * sin(yaw)
            # v_east  = v_forward * sin(yaw) + v_right * cos(yaw)
            
            # nav_vel[0] is forward (x), nav_vel[1] is right (-y)
            vel_forward = self.nav_vel[0]
            vel_right = self.nav_vel[1] 

            v_north = vel_forward * cos_yaw - vel_right * sin_yaw
            v_east  = vel_forward * sin_yaw + vel_right * cos_yaw
            v_down = 0.0 # Pertahankan ketinggian (PX4 auto hold altitude di velocity mode 0 z)

            traj_msg.velocity = [float(v_north), float(v_east), v_down]
            traj_msg.yawspeed = float(self.nav_vel[2])

        self.traj_pub.publish(traj_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CmdSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
