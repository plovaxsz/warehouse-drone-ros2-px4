#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import time

def main():
    rclpy.init()
    navigator = BasicNavigator()

    waypoints_coords = [
        [3.0, 0.0],  # Tengah lorong
        [6.0, 0.0],  # Ujung lorong
        [0.0, 0.0]   # Pulang
    ]

    print("\n🚀 STARTING AUTONOMOUS MAPPING & SURVEILLANCE")
    
    # Tunggu Nav2 aktif sepenuhnya
    navigator.lifecycleStartup()

    # Set Initial Pose (Asumsi mulai dari 0,0)
    print("🔹 Setting Initial Pose (0,0)...")
    init_pose = PoseStamped()
    init_pose.header.frame_id = 'map'
    init_pose.header.stamp = navigator.get_clock().now().to_msg()
    init_pose.pose.position.x = 0.0
    init_pose.pose.position.y = 0.0
    init_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(init_pose)
    
    print("⏳ Waiting for initial map update (5s)...")
    time.sleep(5)

    for i, pt in enumerate(waypoints_coords):
        print(f"\n📍 [TARGET {i+1}] Moving to: {pt} ...")
        
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = navigator.get_clock().now().to_msg()
        goal.pose.position.x = float(pt[0])
        goal.pose.position.y = float(pt[1])
        goal.pose.orientation.w = 1.0
        
        navigator.goToPose(goal)

        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback:
               print(f"   >>> Distance remaining: {feedback.distance_remaining:.2f} m", end='\r')
            time.sleep(0.5)

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"\n✅ Waypoint {i+1} Reached! Scanning area...")
            time.sleep(2)
        else:
            print(f"\n⚠️ Waypoint {i+1} Failed/Aborted!")

    print("\n🏁 MISSION COMPLETE.")
    exit(0)

if __name__ == '__main__':
    main()
