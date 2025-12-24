#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from px4_msgs.msg import TrajectorySetpoint, OffboardControlMode, VehicleCommand, VehicleStatus, VehicleOdometry
import numpy as np

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')
        
        # PENTING: Waktu Simulasi
        self.declare_parameter('use_sim_time', True)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.cb_status, qos)
        self.odom_sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.cb_odom, qos)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cb_cmd_vel, 10)

        self.pub_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.pub_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.pub_vehicle_command = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)

        self.nav_state = 0
        self.arming_state = 0
        self.current_yaw = 0.0
        self.vel_cmd = [0.0, 0.0, 0.0]
        self.yaw_rate_cmd = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.flight_height = -2.0 # Terbang 2 meter
        
        # Timer Heartbeat (Cepat)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.arm_counter = 0

    def cb_status(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def cb_odom(self, msg):
        q = msg.q
        siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
        self.current_yaw = np.arctan2(siny_cosp, cosy_cosp)

    def cb_cmd_vel(self, msg):
        self.last_cmd_time = self.get_clock().now()
        # Transformasi Nav2 (Body) -> PX4 (World)
        cy = np.cos(self.current_yaw)
        sy = np.sin(self.current_yaw)
        vx = msg.linear.x
        vy = msg.linear.y
        self.vel_cmd[0] = vx * cy - vy * sy
        self.vel_cmd[1] = vx * sy + vy * cy
        self.yaw_rate_cmd = -msg.angular.z

    def publish_command(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = p1
        msg.param2 = p2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_vehicle_command.publish(msg)

    def timer_callback(self):
        # 1. Heartbeat
        mode = OffboardControlMode()
        mode.position = False
        mode.velocity = True
        mode.acceleration = False
        mode.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_offboard_mode.publish(mode)

        # 2. Logic Auto-Takeoff (Agresif)
        if self.arm_counter < 50: 
            self.arm_counter += 1
            return

        # Paksa Offboard & Arming
        if self.nav_state != 14: # 14 = Offboard
            self.publish_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        
        if self.arming_state != 2: # 2 = Armed
            self.publish_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

        # 3. Kirim Perintah Gerak
        traj = TrajectorySetpoint()
        traj.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        # Cek apakah Nav2 diam?
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        
        if elapsed > 0.5: # Jika tidak ada perintah Nav2
            traj.velocity = [0.0, 0.0, 0.0]
            traj.position = [float('nan'), float('nan'), self.flight_height] # Hover diam
            traj.yaw = float('nan')
        else: # Jika ada perintah Nav2
            traj.velocity = [self.vel_cmd[0], self.vel_cmd[1], 0.0]
            traj.position = [float('nan'), float('nan'), self.flight_height]
            traj.yawspeed = self.yaw_rate_cmd

        self.pub_trajectory.publish(traj)

def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
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
