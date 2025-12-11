#!/usr/bin/env python
import rospy
import math
import numpy as np
import csv
import os
import sys
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Float32MultiArray, Bool
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from ackermann_msgs.msg import AckermannDrive

# --- CONFIGURATION ---
# Try to match the car name from args or default to car_8
try:
    CAR_NAME = str(sys.argv[1])
except IndexError:
    CAR_NAME = "car_8"

# Tuning
LOOKAHEAD_DIST = 1.2    # Lookahead for the Overtake Controller
WHEELBASE_LEN = 0.325
OBSTACLE_DIST_THRESH = 1.5  # If obstacle is closer than this, we react
LANE_OFFSET = 0.5       # Width of the lanes (Left/Right)
COLLISION_WIDTH = 0.4   # Safety radius around path points

class DistFinder:
    def __init__(self):
        rospy.init_node("dist_finder")
        
        # --- SUBSCRIBERS ---
        self.scan_sub = rospy.Subscriber(f'/{CAR_NAME}/scan', LaserScan, self.scan_callback)
        self.pose_sub = rospy.Subscriber(f'/{CAR_NAME}/particle_filter/viz/inferred_pose', PoseStamped, self.pose_callback)
        
        # --- PUBLISHERS ---
        # 1. Visualization
        self.path_pub = rospy.Publisher(f'/{CAR_NAME}/dist_finder/selected_path', Path, queue_size=1)
        self.marker_pub = rospy.Publisher(f'/{CAR_NAME}/dist_finder/candidates', MarkerArray, queue_size=1)
        
        # 2. Control (The "Takeover" Logic)
        self.safety_pub = rospy.Publisher(f'/{CAR_NAME}/safety_override', Bool, queue_size=1)
        self.drive_pub  = rospy.Publisher(f'/{CAR_NAME}/offboard/command', AckermannDrive, queue_size=1)

        # --- STATE ---
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.center_line = []
        self.candidate_paths = [] # List of (path_array, offset_name)
        self.active_offset_idx = 1 # Start with Center (Index 1)
        
        # Load the map immediately
        self.load_raceline()
        self.generate_lanes()
        
        rospy.loginfo(f"DistFinder initialized for {CAR_NAME}. Waiting for Pose/Scan...")

    def load_raceline(self):
        # Tries to find the file in common locations
        traj_name = "base_map_3_raceline" # Update this if your pure_pursuit uses a different default
        
        home = os.path.expanduser('~')
        paths = [
            f"{home}/depend_ws/src/f1tenth_purepursuit/path/{traj_name}.csv",
            f"{home}/catkin_ws/src/f1tenth_purepursuit/path/{traj_name}.csv",
            f"{traj_name}.csv" 
        ]
        
        file_path = ""
        for p in paths:
            if os.path.exists(p):
                file_path = p
                break
                
        if not file_path:
            rospy.logerr("Raceline file not found! DistFinder cannot generate offsets.")
            return

        try:
            with open(file_path) as csv_file:
                csv_reader = csv.reader(csv_file, delimiter=',')
                for waypoint in csv_reader:
                    self.center_line.append([float(waypoint[0]), float(waypoint[1])])
            self.center_line = np.array(self.center_line)
            rospy.loginfo(f"Loaded {len(self.center_line)} waypoints.")
        except Exception as e:
            rospy.logerr(f"Error loading CSV: {e}")

    def generate_lanes(self):
        if len(self.center_line) == 0: return

        # We generate 3 paths: [Right, Center, Left]
        # Offset + is Left, - is Right (standard coordinate frame)
        offsets = [-LANE_OFFSET, 0.0, LANE_OFFSET]
        names = ["Right", "Center", "Left"]
        
        for i, off in enumerate(offsets):
            if off == 0.0:
                self.candidate_paths.append((self.center_line, names[i]))
            else:
                new_path = self.generate_offset_path(self.center_line, off)
                self.candidate_paths.append((new_path, names[i]))

    def generate_offset_path(self, path, offset):
        new_path = []
        n_points = len(path)
        for i in range(n_points):
            p1 = path[i]
            p2 = path[(i+1) % n_points]
            
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            yaw = math.atan2(dy, dx)
            norm_yaw = yaw + math.pi/2
            
            nx = offset * math.cos(norm_yaw)
            ny = offset * math.sin(norm_yaw)
            
            new_path.append([p1[0] + nx, p1[1] + ny])
        return np.array(new_path)

    def pose_callback(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        q = msg.pose.orientation
        self.heading = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))[2]

    def scan_callback(self, data):
        if len(self.candidate_paths) == 0: return
        
        # 1. Check validity of all lanes
        # valid_flags will be like [True, False, True] (Right clear, Center blocked, Left clear)
        valid_flags = []
        for path, name in self.candidate_paths:
            is_clear = self.check_path_collision(path, data)
            valid_flags.append(is_clear)

        # 2. Decision Logic
        # Priority: Stick to current lane if valid. If not, switch to Center > Left > Right (preference)
        
        # Check current lane first to prevent flickering
        current_valid = valid_flags[self.active_offset_idx]
        
        target_idx = self.active_offset_idx
        
        if not current_valid:
            # Current path blocked! Find best alternative.
            # Prefer Center (idx 1) -> Left (idx 2) -> Right (idx 0)
            if valid_flags[1]:     target_idx = 1 # Center
            elif valid_flags[2]:   target_idx = 2 # Left
            elif valid_flags[0]:   target_idx = 0 # Right
            else:
                # All paths blocked. Emergency. Keep Center and brake.
                target_idx = 1
                rospy.logwarn("ALL LANES BLOCKED!")

        # Hysteresis: Only switch if we are sure (could add a timer here, but simple logic for now)
        self.active_offset_idx = target_idx
        
        # 3. Control Handover
        # If we are on the Center Line (Index 1) AND it is clear, let PurePursuit.py drive.
        # Otherwise (Overtaking OR Center blocked), WE drive.
        
        is_center_optimal = (target_idx == 1)
        
        if is_center_optimal and valid_flags[1]:
            # Normal condition: Path clear, stay on optimal line.
            self.safety_pub.publish(False) # Unmute pure_pursuit.py
        else:
            # OVERTAKE MODE
            self.safety_pub.publish(True) # Mute pure_pursuit.py
            
            # Calculate our own drive command on the offset path
            selected_path = self.candidate_paths[target_idx][0]
            steer, speed = self.get_pure_pursuit_command(selected_path)
            
            drive_msg = AckermannDrive()
            drive_msg.steering_angle = steer
            drive_msg.speed = speed
            self.drive_pub.publish(drive_msg)
            
            rospy.loginfo_throttle(0.5, f"OVERTAKING: Taking control on {self.candidate_paths[target_idx][1]} Lane")

        # 4. Visualization
        self.publish_viz(valid_flags, target_idx)

    def check_path_collision(self, path, scan):
        # Simple collision check:
        # Get points on path in front of car (e.g. 0m to 3m ahead)
        # Check if any LiDAR point is close to these path points
        
        # Optimization: Just look at the "Lookahead Point" and 2 points before/after it
        # Real collision checking is expensive in python, this is a heuristic:
        # "Is there a lidar point within X meters of the path's lookahead?"
        
        ranges = np.array(scan.ranges)
        ranges[np.isnan(ranges)] = scan.range_max
        ranges[np.isinf(ranges)] = scan.range_max
        
        # Convert scan to Cartesian (Local Frame)
        angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))
        # Filter for valid points (ignore far away ones to save compute)
        valid_mask = ranges < OBSTACLE_DIST_THRESH
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        
        if len(valid_ranges) == 0: return True # Clear
        
        lx = valid_ranges * np.cos(valid_angles)
        ly = valid_ranges * np.sin(valid_angles)
        
        # Transform active path segment to local frame? 
        # Easier: Transform Lidar points to Map frame and check distance to path.
        
        # Map Frame Lidar Points
        c, s = math.cos(self.heading), math.sin(self.heading)
        mx = self.x + (c * lx - s * ly)
        my = self.y + (s * lx + c * ly)
        
        # Find closest path point to the car
        dists = np.linalg.norm(path[:, :2] - np.array([self.x, self.y]), axis=1)
        closest_idx = np.argmin(dists)
        
        # Check collision for the next 20 points (approx 2-3 meters)
        check_horizon = 20 
        path_segment = path[closest_idx : closest_idx + check_horizon]
        
        for px, py in path_segment:
            # Check distance from this path point to ALL lidar points
            # This is O(N*M), slow in Python.
            # Faster: Check if any lidar point is within Radius of this path point
            d_sq = (mx - px)**2 + (my - py)**2
            if np.any(d_sq < COLLISION_WIDTH**2):
                return False # Collision detected
                
        return True # Clear

    def get_pure_pursuit_command(self, path):
        # 1. Find closest point
        dists = np.linalg.norm(path[:, :2] - np.array([self.x, self.y]), axis=1)
        closest_idx = np.argmin(dists)
        
        # 2. Find Lookahead
        target_idx = closest_idx
        path_len = len(path)
        for i in range(closest_idx, closest_idx + path_len):
            idx = i % path_len
            dist = np.linalg.norm(path[idx] - np.array([self.x, self.y]))
            if dist > LOOKAHEAD_DIST:
                target_idx = idx
                break
                
        target = path[target_idx]
        
        # 3. Calculate Steering
        dx = target[0] - self.x
        dy = target[1] - self.y
        
        x_rel = dx * math.cos(self.heading) + dy * math.sin(self.heading)
        y_rel = -dx * math.sin(self.heading) + dy * math.cos(self.heading)
        
        alpha = math.atan2(y_rel, x_rel)
        L = math.sqrt(dx**2 + dy**2)
        
        steer = math.atan2(2.0 * WHEELBASE_LEN * math.sin(alpha), L)
        steer = max(-0.41, min(0.41, steer)) # Clamp
        
        # Speed Control: Slow down if we are overtaking/swerving
        speed = 2.0 if abs(steer) > 0.2 else 4.0 # Simple dynamic speed
        
        # Scale steering to servo range [-100, 100] if your car needs it
        # Assuming your Ackermann msg expects radians, but based on pure_pursuit.py:
        steer_out = (steer / 0.4189) * 100.0
        
        # Speed scaling (match your pure_pursuit logic roughly)
        speed_out = 25.0 if abs(steer_out) < 15 else 12.0
        
        return steer_out, speed_out

    def publish_viz(self, valid_flags, selected_idx):
        # Publish the path
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()
        active_path = self.candidate_paths[selected_idx][0]
        
        # Optimization: Only publish local path to save bandwidth
        dists = np.linalg.norm(active_path[:, :2] - np.array([self.x, self.y]), axis=1)
        closest = np.argmin(dists)
        pts = active_path[closest : closest+50]
        
        for p in pts:
            pose = PoseStamped()
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

        # Publish Candidate Lines (Red=Blocked, Green=Clear)
        m_array = MarkerArray()
        for i, (path, name) in enumerate(self.candidate_paths):
            m = Marker()
            m.header.frame_id = "map"
            m.id = i
            m.type = Marker.LINE_STRIP
            m.scale.x = 0.05
            m.color.a = 0.8
            
            if i == selected_idx:
                m.scale.x = 0.1 # Thicker for active
                m.color.b = 1.0 # Blue for selected
            elif valid_flags[i]:
                m.color.g = 1.0 # Green for valid
            else:
                m.color.r = 1.0 # Red for blocked
                
            # Localviz
            dists = np.linalg.norm(path[:, :2] - np.array([self.x, self.y]), axis=1)
            closest = np.argmin(dists)
            pts = path[max(0, closest-10) : min(len(path), closest+80)]
            
            for pt in pts:
                p = Point(); p.x = pt[0]; p.y = pt[1]
                m.points.append(p)
            m_array.markers.append(m)
            
        self.marker_pub.publish(m_array)

if __name__ == "__main__":
    try:
        DistFinder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass