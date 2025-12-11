#!/usr/bin/env python
import rospy
import os
import sys
import csv
import math
import tf
import numpy as np
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PolygonStamped, Point32, PoseStamped
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker 
from std_msgs.msg import Bool

# --- TUNING PARAMETERS ---
# CRITICAL: Keep LOOKAHEAD_MIN high enough (1.0m+) to prevent oscillation
LOOKAHEAD_MIN = 1.0   
LOOKAHEAD_MAX = 3.0   

# Velocity Factor: Speed (approx 45) * 0.035 = ~1.5m lookahead
# Lower this if it cuts corners too much. Raise it if it wobbles on straights.
LOOKAHEAD_GAIN = 0.035 

WHEELBASE_LEN = 0.325
STEERING_RANGE = 100.0 
MAX_STEERING_ANGLE = 0.4189 

# Speed Output Settings
SPEED_MAX = 65.0 # Straights
SPEED_MIN = 25.0 # Corners

class PurePursuit:
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        # --- ARGS ---
        try:
            self.car_name = str(sys.argv[1])
            self.trajectory_name = str(sys.argv[2])
        except IndexError:
            self.car_name = "car_8"
            self.trajectory_name = "base_map_3_raceline"

        # --- STATE ---
        self.plan = []
        self.last_closest_index = 0
        self.avg_velocity = 0.0 # <--- THE FIX: Smoothed velocity
        self.safety_override = False

        # --- ROS TOPICS ---
        self.command_pub = rospy.Publisher(f'/{self.car_name}/offboard/command', AckermannDrive, queue_size=1)
        self.polygon_pub = rospy.Publisher(f'/{self.car_name}/purepursuit_control/visualize', PolygonStamped, queue_size=1)
        self.marker_pub  = rospy.Publisher(f'/{self.car_name}/purepursuit_control/target_marker', Marker, queue_size=1)
        self.path_pub    = rospy.Publisher(f'/{self.car_name}/purepursuit_control/path', Path, queue_size=1, latch=True)

        # Subscribers
        rospy.Subscriber(f'/{self.car_name}/particle_filter/viz/inferred_pose', PoseStamped, self.control_callback)
        rospy.Subscriber(f'/{self.car_name}/odom', Odometry, self.odom_callback)
        rospy.Subscriber(f'/{self.car_name}/safety_override', Bool, self.safety_callback)

        self.construct_path()
        rospy.spin()

    def safety_callback(self, msg):
        self.safety_override = msg.data

    def odom_callback(self, data):
        # SMOOTHING LOGIC:
        # Instead of taking raw speed, we blend it: 80% old average, 20% new reading
        raw_speed = data.twist.twist.linear.x
        self.avg_velocity = (0.8 * self.avg_velocity) + (0.2 * raw_speed)

    def construct_path(self):
        # Load CSV
        home = os.path.expanduser('~')
        paths = [
            f"{home}/depend_ws/src/f1tenth_purepursuit/path/{self.trajectory_name}.csv",
            f"{home}/catkin_ws/src/f1tenth_purepursuit/path/{self.trajectory_name}.csv",
            f"{self.trajectory_name}.csv"
        ]
        file_path = ""
        for p in paths:
            if os.path.exists(p):
                file_path = p; break
        
        if not file_path:
            rospy.logerr("CSV Not Found!"); return

        with open(file_path) as f:
            reader = csv.reader(f)
            for row in reader:
                self.plan.append([float(row[0]), float(row[1])])
        
        # Publish Path Viz
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        for p in self.plan:
            ps = PoseStamped()
            ps.pose.position.x = p[0]; ps.pose.position.y = p[1]; ps.pose.orientation.w=1.0
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    def control_callback(self, data):
        # 1. Safety Check
        if self.safety_override or not self.plan:
            return

        # 2. Car State
        ox = data.pose.position.x
        oy = data.pose.position.y
        # Quat -> Yaw
        q = data.pose.orientation
        heading = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))[2]

        # 3. Find Closest Point (Windowed)
        min_dist_sq = float('inf')
        closest_idx = self.last_closest_index
        path_len = len(self.plan)
        
        # Search -10 to +50 points
        for i in range(self.last_closest_index - 10, self.last_closest_index + 50):
            idx = i % path_len
            dx = ox - self.plan[idx][0]
            dy = oy - self.plan[idx][1]
            d_sq = dx*dx + dy*dy
            if d_sq < min_dist_sq:
                min_dist_sq = d_sq
                closest_idx = idx
        
        # Failsafe: If lost (>3m away), search whole track
        if min_dist_sq > 9.0:
            for i in range(path_len):
                dx = ox - self.plan[i][0]; dy = oy - self.plan[i][1]
                d_sq = dx*dx + dy*dy
                if d_sq < min_dist_sq:
                    min_dist_sq = d_sq; closest_idx = i
                    
        self.last_closest_index = closest_idx

        # 4. Dynamic Lookahead (USING SMOOTHED VELOCITY)
        lookahead = self.avg_velocity * LOOKAHEAD_GAIN
        lookahead = max(LOOKAHEAD_MIN, min(lookahead, LOOKAHEAD_MAX))

        # 5. Find Target Point
        target_idx = closest_idx
        found = False
        
        for i in range(closest_idx, closest_idx + path_len):
            idx = i % path_len
            dx = self.plan[idx][0] - ox
            dy = self.plan[idx][1] - oy
            dist = math.sqrt(dx*dx + dy*dy)
            if dist > lookahead:
                # Geometric Check: Is this point actually in front of us?
                # Transform to local frame
                lx = dx * math.cos(heading) + dy * math.sin(heading)
                if lx > 0: # Only accept points in front
                    target_idx = idx
                    found = True
                    break
        
        if not found:
            target_idx = (closest_idx + 15) % path_len
            
        tx = self.plan[target_idx][0]
        ty = self.plan[target_idx][1]

        # 6. Calculate Steering
        dx = tx - ox
        dy = ty - oy
        
        # Transform to Vehicle Frame
        x_rel = dx * math.cos(heading) + dy * math.sin(heading)
        y_rel = -dx * math.sin(heading) + dy * math.cos(heading)
        
        alpha = math.atan2(y_rel, x_rel)
        L = math.sqrt(dx*dx + dy*dy)
        
        steer_rad = math.atan2(2.0 * WHEELBASE_LEN * math.sin(alpha), L)
        steer_rad = max(-MAX_STEERING_ANGLE, min(MAX_STEERING_ANGLE, steer_rad))
        
        # Output Command
        cmd = AckermannDrive()
        cmd.steering_angle = (steer_rad / MAX_STEERING_ANGLE) * STEERING_RANGE
        
        # 7. Speed Logic (Threshold)
        turn_intensity = abs(cmd.steering_angle) / STEERING_RANGE
        if turn_intensity < 0.15:
            cmd.speed = SPEED_MAX
        elif turn_intensity < 0.4:
            cmd.speed = (SPEED_MAX + SPEED_MIN) / 2
        else:
            cmd.speed = SPEED_MIN
            
        self.command_pub.publish(cmd)
        
        # 8. Visualization
        self.publish_viz(ox, oy, tx, ty)

    def publish_viz(self, ox, oy, tx, ty):
        # Red Sphere Target
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.SPHERE; marker.action = Marker.ADD
        marker.pose.position.x = tx; marker.pose.position.y = ty
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5; marker.scale.y = 0.5; marker.scale.z = 0.5
        marker.color.a = 1.0; marker.color.r = 1.0
        self.marker_pub.publish(marker)

if __name__ == '__main__':
    PurePursuit()