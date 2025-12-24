#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleAttitude
from rclpy.parameter import Parameter
import numpy as np

class CmdSender(Node):
    def __init__(self):
        super().__init__('cmd_sender')
        
        # FORCE SIM TIME
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)
        
        self.create_subscription(Twist, '/cmd_vel', self.cb_vel, 10)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.cb_att, qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.cb_status, qos)
        
        self.pub_offboard = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.pub_traj = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        
        self.yaw = 0.0
        self.is_offboard = False
        self.create_timer(0.1, self.timer_cb)
        self.get_logger().info("✅ Cmd Sender Active (Sim Time Forced)")

    def cb_status(self, msg): self.is_offboard = (msg.nav_state == 14)
    
    def cb_att(self, msg):
        q = msg.q
        self.yaw = np.arctan2(2.0*(q[0]*q[3] + q[1]*q[2]), 1.0 - 2.0*(q[2]**2 + q[3]**2))

    def timer_cb(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        self.pub_offboard.publish(msg)

    def cb_vel(self, msg):
        # Coordinate Transform: Body (ENU) -> Local (NED)
        cos_y = np.cos(self.yaw)
        sin_y = np.sin(self.yaw)
        
        # ROS Body X (Forward) -> PX4 North/East logic
        vx = msg.linear.x * cos_y - msg.linear.y * sin_y
        vy = msg.linear.x * sin_y + msg.linear.y * cos_y

        traj = TrajectorySetpoint()
        traj.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        traj.position = [float('nan'), float('nan'), float('nan')]
        traj.velocity = [float(vx), float(vy), 0.0]
        traj.yaw = float('nan')
        traj.yawspeed = float(-msg.angular.z)
        self.pub_traj.publish(traj)

def main():
    rclpy.init()
    rclpy.spin(CmdSender())

if __name__ == '__main__':
    main()
