#!/usr/bin/env python
import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Point, PoseStamped, PointStamped
from visualization_msgs.msg import Marker
import tf

# --- CONFIGURATION ---
LOOKAHEAD_DIST = 1.0
BUBBLE_RADIUS = 0.5  # Size of the "safety bubble" around obstacles
WALL_FOLLOW_BIAS = 0.0 # 0.0 = pure pursuit bias, 1.0 = pure gap bias

class HybridRacer:
    def __init__(self):
        rospy.init_node('hybrid_racer')
        
        # Topics
        self.scan_sub = rospy.Subscriber('/car_8/scan', LaserScan, self.scan_callback)
        self.pose_sub = rospy.Subscriber('/car_8/particle_filter/viz/inferred_pose', PoseStamped, self.pose_callback)
        
        # Publishers
        self.drive_pub = rospy.Publisher('/car_8/offboard/command', AckermannDrive, queue_size=1)
        self.marker_pub = rospy.Publisher('/car_8/debug_marker', Marker, queue_size=1)
        
        # State
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.waypoints = []
        
        # Load Waypoints (Reuse your CSV loading logic from pure_pursuit.py here)
        # self.load_waypoints("my_race_line.csv")
        # Dummy waypoints for example
        self.waypoints = np.array([[0,0], [10,0], [20, 5]]) 

        self.listener = tf.TransformListener()

    def pose_callback(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        # Convert quaternion to yaw
        quaternion = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        self.heading = tf.transformations.euler_from_quaternion(quaternion)[2]

    def get_pure_pursuit_angle(self):
        # 1. Find closest point on path
        # 2. Find lookahead point
        # 3. Calculate steering angle required to hit lookahead point
        # (Paste logic from your pure_pursuit.py TODO 1-4 here)
        
        # For this example, let's assume we found the goal point in car frame:
        # goal_x_local, goal_y_local
        
        # Placeholder return (radians)
        return 0.0 

    def preprocess_lidar(self, ranges):
        # Basic cleaning (inf/nan removal)
        proc_ranges = np.array(ranges)
        proc_ranges[np.isinf(proc_ranges)] = 10.0
        proc_ranges[np.isnan(proc_ranges)] = 10.0
        return proc_ranges

    def scan_callback(self, data):
        ranges = self.preprocess_lidar(data.ranges)
        
        # --- STEP 1: Pure Pursuit (The "Desire") ---
        pp_angle = self.get_pure_pursuit_angle()
        
        # --- STEP 2: Follow the Gap (The "Constraint") ---
        # Find closest obstacle to create bubble
        min_dist_idx = np.argmin(ranges)
        min_dist = ranges[min_dist_idx]
        
        # Disparity Extender / Safety Bubble Logic
        # (Use your logic from dist_finder.py to zero out ranges around obstacles)
        # Simply setting a bubble around the closest point for brevity:
        angle_inc = data.angle_increment
        bubble_idx_width = int(BUBBLE_RADIUS / (min_dist * angle_inc))
        
        start = max(0, min_dist_idx - bubble_idx_width)
        end = min(len(ranges), min_dist_idx + bubble_idx_width)
        ranges[start:end] = 0.0 # "Mask" the obstacle
        
        # Find "Free Space" gaps (thresholding)
        threshold = 2.0 # Meters
        gap_mask = ranges > threshold
        
        # --- STEP 3: The Integration (Optimization) ---
        # Instead of finding the "Widest" gap, find the gap closest to PP angle.
        
        best_idx = -1
        min_angle_diff = float('inf')
        
        # Iterate through valid gap indices
        for i, is_free in enumerate(gap_mask):
            if is_free:
                # Calculate angle of this specific lidar beam
                beam_angle = data.angle_min + (i * data.angle_increment)
                
                # Calculate difference from our Pure Pursuit Desired Angle
                diff = abs(beam_angle - pp_angle)
                
                # If this beam is closer to our desire, pick it
                if diff < min_angle_diff:
                    min_angle_diff = diff
                    best_idx = i
                    
        # If no gap found, fallback to safety (stop or simple FTG)
        if best_idx == -1:
            steering_angle = 0.0
            speed = 0.0
        else:
            steering_angle = data.angle_min + (best_idx * data.angle_increment)
            speed = 4.0 # Set dynamic speed here based on steering angle
            
        # Publish Command
        drive = AckermannDrive()
        drive.speed = speed
        drive.steering_angle = steering_angle # Check radians vs degrees conversion!
        self.drive_pub.publish(drive)
        
        # --- VISUALIZATION ---
        self.publish_debug_markers(pp_angle, steering_angle)

    def publish_debug_markers(self, pp_angle, actual_angle):
        # Visualizes the conflict between "Desire" (PP) and "Reality" (FTG)
        marker = Marker()
        marker.header.frame_id = "map" # Car frame
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 1.0 # Arrow length
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0 # Green = Actual Command
        marker.color.b = 0.0
        
        # Convert angle to quaternion for marker orientation
        q = tf.transformations.quaternion_from_euler(0, 0, actual_angle)
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]
        
        self.marker_pub.publish(marker)

if __name__ == '__main__':
    try:
        HybridRacer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass