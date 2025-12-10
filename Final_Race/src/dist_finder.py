#!/usr/bin/env python
import rospy
import math
import numpy as np
import csv
import os
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path

# --- CONFIGURATION ---
CAR_NAME = "car_8"
RACELINE_FILE = "base_map_2_raceline.csv" # Assumes it"s in the same folder structure as pure_pursuit
LOOKAHEAD_DIST = 1.5 # Meters
WHEELBASE_LEN = 0.325
OBSTACLE_THRESHOLD = 1.0 # Meters (Distance to consider a point "blocked")
LANE_OFFSET = 0.6 # Meters (Distance between parallel lanes)

class DistFinder:
    def __init__(self):
        rospy.init_node("dist_finder")
        
        # Topics
        self.scan_sub = rospy.Subscriber('/car_8/scan', LaserScan, self.scan_callback)
        self.pose_sub = rospy.Subscriber('/car_8/particle_filter/viz/inferred_pose', PoseStamped, self.pose_callback)
        
        self.gap_pub = rospy.Publisher('/car_8/gap_info', Float32MultiArray, queue_size=1)
        self.path_pub = rospy.Publisher('/car_8/selected_path', Path, queue_size=1)
        self.marker_pub = rospy.Publisher('/car_8/debug_markers', MarkerArray, queue_size=1)
        self.closest_pub = rospy.Publisher('/car_8/closest_marker', Marker, queue_size=1)

        # State
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.current_speed = 0.0 # We might need odom for this if we want dynamic lookahead
        self.center_line = []
        # Racelines
        # List of (path, offset_value) tuples
        # We generate multiple granular offsets to find a "valid" line that doesn"t hit the wall
        self.candidate_paths = [] 
        self.last_selected_offset = 0.0 # State for stability 
        
        self.load_and_generate_racelines()
        
        self.listener = tf.TransformListener()

    def load_and_generate_racelines(self):
        # 1. Load Center Line
        # Try to find the file in the standard location
        file_path = os.path.expanduser("~/depend_ws/src/f1tenth_purepursuit/path/base_map_2_raceline.csv")
        if not os.path.exists(file_path):
            rospy.logwarn("Raceline file not found. Trying local directory.")
            file_path = RACELINE_FILE # Fallback
            
        try:
            with open(file_path) as csv_file:
                csv_reader = csv.reader(csv_file, delimiter=",")
                for waypoint in csv_reader:
                    self.center_line.append([float(waypoint[0]), float(waypoint[1])]) # x, y
            rospy.loginfo("Loaded waypoints.")
        except Exception as e:
            rospy.logerr(e) 
            rospy.logerr("Failed to load raceline: e")
            return

        self.center_line = np.array(self.center_line)
        
        # 2. Generate Candidate Lines
        # Instead of just one Left/Right, we generate a spread of offsets.
        # If the optimal line is near the wall, the large offsets will be invalid (hit wall),
        # but the smaller offsets might still be valid.
        
        # Offsets in meters. 0.0 is the optimal line.
        # We prioritize small deviations over large ones.
        offsets = [-0.3, 0.0, 0.3]
        
        for off in offsets:
            if off == 0.0:
                self.candidate_paths.append((self.center_line, 0.0))
            else:
                # Generate offset path
                new_path = self.generate_offset(self.center_line, off)
                self.candidate_paths.append((new_path, off))
        
        rospy.loginfo("Generated candidate racelines.")
        
    def generate_offset(self, path, offset):
        new_path = []
        for i in range(len(path)):
            # Get tangent vector
            p1 = path[i]
            p2 = path[(i+1) % len(path)]
            
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            yaw = math.atan2(dy, dx)
            
            # Perpendicular vector (+90 degrees)
            norm_yaw = yaw + math.pi/2
            
            nx = offset * math.cos(norm_yaw)
            ny = offset * math.sin(norm_yaw)
            
            new_path.append([p1[0] + nx, p1[1] + ny]) # Keep same velocity
            
        return np.array(new_path)

    def pose_callback(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        quaternion = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        self.heading = tf.transformations.euler_from_quaternion(quaternion)[2]

    def preprocess_lidar(self, ranges):
        # Clean up the LiDAR data
        # 1. Convert tuple to list/numpy array
        proc_ranges = np.array(ranges)
        
        # 2. Handle Inf/NaN
        # In many sims, 'inf' means "too far". We set it to max range.
        proc_ranges[np.isinf(proc_ranges)] = 5.0
        proc_ranges[np.isnan(proc_ranges)] = 5.0
        
        # 3. Handle 0.0 (Often means "too close" or "error", but in some sims means "max range")
        # If track is empty and we see 0.0, it's likely noise/max range.
        # SAFE default: treat as max range unless we are sure it's an obstacle.
        # But if it really IS an obstacle touching the sensor, this is dangerous.
        # Compromise: Set to small non-zero or max?
        # Given "empty track" user report, 0.0 is likely "max range" error.
        proc_ranges[proc_ranges == 0.0] = 5.0
        
        return proc_ranges

    def scan_callback(self, data):
        if len(self.candidate_paths) == 0:
            return

        # Preprocess ranges
        ranges = self.preprocess_lidar(data.ranges)
        
        # Create a dummy data object to pass cleaned ranges to check functions
        # (Or update valid check to take ranges directly)
        
        # ... logic continues ...

        # 1. Check Collisions & Select Best Line
        # We iterate through candidates in order of priority (smallest offset first).
        # The first one that is "valid" (no collisions) is chosen.
        
        selected_path = None
        selected_offset = 0.0

        min_dist, angle_of_closest = self.closest_obstacle(data)  # For debugging/logging

        # 3. Determine direction
        if angle_of_closest > 20 and min_dist <= 0.5:
            # Obstacle is on the LEFT
            print("LEFT| angle: " + str(angle_of_closest) + " dist: " + str(min_dist))
            selected_path = selected_path = self.candidate_paths[2][0] 
        elif angle_of_closest < -20 and min_dist <= 0.5:
            # Obstacle is on the RIGHT
            print("RIGHT| angle: " + str(angle_of_closest) + " dist: " + str(min_dist))
            selected_path = self.candidate_paths[0][0] 
        else:
            # Obstacle is dead CENTER (exactly 90)
            print("Ahead| angle: " + str(angle_of_closest) + " dist: " + str(min_dist))

        print("---------------------")
        
        # Fallback: If ALL are blocked, pick the center line (offset 0) and hope/brake.
        if selected_path is None:
            #rospy.logwarn("ALL PATHS BLOCKED! Defaulting to Center.")
            selected_path = self.candidate_paths[1][0] # The 0.0 offset path
            selected_offset = 0.0
        
        # Debug: Publish closest obstacle marker
        self.publish_obstacle_marker(data.header, min_dist, angle_of_closest)

        # 3. Calculate Pure Pursuit Steering
        steering_angle = self.get_pure_pursuit_command(selected_path)
        
        # 4. Publish Gap Info (Steering Angle + Distance/Speed)
        # Using the gap_info format: [steering_angle_rad, target_velocity]
        msg = Float32MultiArray()
        msg.data = [steering_angle]
        self.gap_pub.publish(msg)
        
        # 5. Visualize
        self.publish_selected_path(selected_path)
        self.publish_all_paths(data, selected_path)

    def closest_obstacle(self,data):
        # 1. Convert ranges to a list for mutability
        ranges = list(data.ranges)
        n = len(ranges)

        # 2. Define the sector of interest (-70 to 70 degrees)
        start_angle = int((math.radians(-70) - data.angle_min) / data.angle_increment)
        end_angle   = int((math.radians( 70) - data.angle_min) / data.angle_increment)

        # 3. Clamp indices to ensure they are within the array bounds
        i_min = max(0, min(n-1, start_angle))
        i_max = max(0, min(n-1, end_angle))
        if i_min > i_max:
            i_min, i_max = i_max, i_min

        # 4. Clean the data: Handle NaNs, Infs, and Cap distances
        # We focus only on the sector we care about to save processing time
        for i in range(i_min, i_max+1):
            if math.isnan(ranges[i]) or math.isinf(ranges[i]):
                # If infinite/nan, set to max range (no obstacle)
                ranges[i] = data.range_max
            elif ranges[i] > data.range_max:
                ranges[i] = data.range_max

        # --- CHANGED SECTION START ---

        # 5. Identify the Closest Obstacle
        # We extract the relevant sector as a numpy array for easy math
        sector_ranges = np.array(ranges[i_min:i_max+1])
        
        # Safety check: if the sector is empty
        if sector_ranges.size == 0:
            return
        
        # Filter out values below range_min (noise/errors)
        # We use boolean indexing to keep only valid readings
        valid_indices = np.where(sector_ranges > data.range_min)[0]
        
        if valid_indices.size == 0:
            # If all data is invalid (too close or errors), assume open space or handle safety
            min_dist = data.range_max
            closest_angle = 0.0 
        else:
            # Get only the valid ranges
            valid_ranges = sector_ranges[valid_indices]
            
            # Robust Minimum: Sort and take average of the closest 3 points to avoid speckle noise
            sorted_ranges = np.sort(valid_ranges)
            if len(sorted_ranges) >= 3:
                min_dist = np.mean(sorted_ranges[:3])
            else:
                min_dist = sorted_ranges[0]

            # Find the index of the value closest to this average minimum in the original array
            # This helps us find the angle roughly corresponding to that distance
            min_idx_local = np.argmin(np.abs(sector_ranges - min_dist))
            target_idx = i_min + min_idx_local
            closest_angle = data.angle_min + target_idx * data.angle_increment

            # 1. Convert the raw radian angle to degrees
            angle_deg = math.degrees(closest_angle)

            # 2. Add the 90-degree offset to match your system 
            # (Standard ROS: 0 is Front. Your System: 0 is Right, 90 is Front)
            adjusted_angle = angle_deg

        return min_dist, adjusted_angle

    def get_pure_pursuit_command(self, path):
        # 1. Find closest point
        dists = np.linalg.norm(path[:, :2] - np.array([self.x, self.y]), axis=1)
        closest_idx = np.argmin(dists)
        
        # 2. Find lookahead point
        target_idx = closest_idx
        for i in range(closest_idx, len(path) + closest_idx): # Handle wrap-around
            idx = i % len(path)
            dist = np.linalg.norm(path[idx, :2] - np.array([self.x, self.y]))
            if dist > LOOKAHEAD_DIST:
                target_idx = idx
                break
        
        target_pt = path[target_idx]
        
        # 3. Calculate Steering
        dx = target_pt[0] - self.x
        dy = target_pt[1] - self.y
        
        x_rel = dx * math.cos(self.heading) + dy * math.sin(self.heading)
        y_rel = -dx * math.sin(self.heading) + dy * math.cos(self.heading)
        
        alpha = math.atan2(y_rel, x_rel)
        actual_lookahead = math.sqrt(dx**2 + dy**2)
        
        steering_angle = math.atan2(2.0 * WHEELBASE_LEN * math.sin(alpha), actual_lookahead)
        
        return steering_angle # Return angle and target velocity

    def publish_selected_path(self, path):
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        
        for pt in path:
            pose = PoseStamped()
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
            
        self.path_pub.publish(msg)

    def publish_all_paths(self, scan_data, selected_path):

        marker_array = MarkerArray()

        for i, (path, offset) in enumerate(self.candidate_paths):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "candidate_paths"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.05 #time width

            # is_valid = self.check_path_validity(path, scan_data, ranges)

            if np.array_equal(path, selected_path):
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                marker.scale.x = 0.1
            
            # elif is_valid:
            #     marker.color.r = 0.0
            #     marker.color.g = 0.0
            #     marker.color.b = 1.0
            #     marker.color.a = 0.5
            else:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.5

            dists = np.linalg.norm(path[:, :2] - np.array([self.x, self.y]), axis=1)
            closest_idx = np.argmin(dists)
            start_viz = max(0, closest_idx - 50)
            end_viz = min(len(path), closest_idx + 150)
            for pt in path[start_viz:end_viz]:
                p = Point()
                p.x = pt[0]
                p.y = pt[1]
                p.z = 0.0
                marker.points.append(p)

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

    def publish_obstacle_marker(self, header, min_dist, angle):
        p_x = min_dist * math.cos(angle)
        p_y = min_dist * math.sin(angle)

        marker = Marker()
        marker.header = header
        marker.ns = "closest_obstacle"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = p_x
        marker.pose.position.y = p_y
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2; marker.scale.y = 0.2; marker.scale.z = 0.2
        marker.color.a = 1.0; marker.color.r = 1.0 
        
        self.closest_pub.publish(marker)


if __name__ == "__main__":
    try:
        DistFinder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
