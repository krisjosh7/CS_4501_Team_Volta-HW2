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
RACELINE_FILE = "base_map_2_raceline.csv"
LOOKAHEAD_DIST = 1.5 
WHEELBASE_LEN = 0.325
OBSTACLE_THRESHOLD = 1.0 
LANE_OFFSET = 0.6 

class DistFinder:
    def __init__(self):
        rospy.init_node("dist_finder")
        
        # Topics
        self.scan_sub = rospy.Subscriber('/car_8/scan', LaserScan, self.scan_callback)
        self.pose_sub = rospy.Subscriber('/car_8/particle_filter/viz/inferred_pose', PoseStamped, self.pose_callback)
        
        self.gap_pub = rospy.Publisher('/car_8/gap_info', Float32MultiArray, queue_size=1)
        self.path_pub = rospy.Publisher('/car_8/selected_path', Path, queue_size=1)
        self.marker_pub = rospy.Publisher('/car_8/debug_markers', MarkerArray, queue_size=1)
        self.collision_pub = rospy.Publisher('/car_8/debug_collision', Marker, queue_size=1)
        # Added a dedicated publisher for the Lookahead point to debug the "Left Turn"
        self.goal_pub = rospy.Publisher('/car_8/pure_pursuit_goal', Marker, queue_size=1)

        # State
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.current_speed = 0.0 
        self.center_line = []
        self.candidate_paths = [] 
        self.last_selected_offset = 0.0 
        self.scan_count = 0 
        self.last_closest_index = 0 
        
        self.load_and_generate_racelines()
        
        self.listener = tf.TransformListener()

    def load_and_generate_racelines(self):
        file_path = os.path.expanduser("~/depend_ws/src/f1tenth_purepursuit/path/base_map_2_raceline.csv")
        if not os.path.exists(file_path):
            rospy.logwarn("Raceline file not found. Trying local directory.")
            file_path = RACELINE_FILE 
            
        try:
            with open(file_path) as csv_file:
                csv_reader = csv.reader(csv_file, delimiter=",")
                for waypoint in csv_reader:
                    self.center_line.append([float(waypoint[0]), float(waypoint[1])]) 
            rospy.loginfo("Loaded waypoints.")
        except Exception as e:
            rospy.logerr(f"Failed to load raceline: {e}")
            return

        self.center_line = np.array(self.center_line)
        
        # Offsets: Optimal (0.0), Left (+0.3), Right (-0.3)
        offsets = [0.0, 0.3, -0.3]
        
        for off in offsets:
            if off == 0.0:
                self.candidate_paths.append((self.center_line, 0.0))
            else:
                new_path = self.generate_offset(self.center_line, off)
                self.candidate_paths.append((new_path, off))
        
        rospy.loginfo("Generated candidate racelines.")
        
    def generate_offset(self, path, offset):
        new_path = []
        for i in range(len(path)):
            p1 = path[i]
            p2 = path[(i+1) % len(path)]
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
        quaternion = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        self.heading = tf.transformations.euler_from_quaternion(quaternion)[2]

    def preprocess_lidar(self, ranges):
        proc_ranges = np.array(ranges)
        proc_ranges[np.isinf(proc_ranges)] = 5.0
        proc_ranges[np.isnan(proc_ranges)] = 5.0
        proc_ranges[proc_ranges == 0.0] = 5.0
        return proc_ranges

    def scan_callback(self, data):
        if len(self.candidate_paths) == 0:
            return

        self.scan_count += 1
        ranges = self.preprocess_lidar(data.ranges)
        
        # --- WINDOWED SEARCH OPTIMIZATION ---
        min_dist_sq = float('inf')
        closest_index = self.last_closest_index
        
        # Search Window: -10 to +50
        start_search = self.last_closest_index - 10
        end_search = self.last_closest_index + 50
        
        for i in range(start_search, end_search):
            idx = i % len(self.center_line)
            dx = self.x - self.center_line[idx][0]
            dy = self.y - self.center_line[idx][1]
            d_sq = dx*dx + dy*dy
            if d_sq < min_dist_sq:
                min_dist_sq = d_sq
                closest_index = idx
                
        # Safety Reset
        if min_dist_sq > 25.0: 
             dists = np.linalg.norm(self.center_line - np.array([self.x, self.y]), axis=1)
             closest_index = np.argmin(dists)
             
        self.last_closest_index = closest_index
        # ------------------------------------

        selected_path = None
        selected_offset = 0.0
        
        def get_path_by_offset(off):
            for p, o in self.candidate_paths:
                if o == off: return p
            return None

        path_0 = get_path_by_offset(0.0)
        path_last = get_path_by_offset(self.last_selected_offset)
        
        if path_0 is not None and self.check_path_validity(path_0, data, ranges, closest_index):
             selected_path = path_0
             selected_offset = 0.0
        elif path_last is not None and self.check_path_validity(path_last, data, ranges, closest_index):
             selected_path = path_last
             selected_offset = self.last_selected_offset
        else:
            for path, offset in self.candidate_paths:
                if self.check_path_validity(path, data, ranges, closest_index):
                    selected_path = path
                    selected_offset = offset
                    break
        
        self.last_selected_offset = selected_offset 
        
        if selected_path is None:
            rospy.logwarn("ALL PATHS BLOCKED! Defaulting to Center.")
            selected_path = self.candidate_paths[0][0] 
            selected_offset = 0.0

        # --- FIX: Pass closest_index to pure pursuit ---
        steering_angle = self.get_pure_pursuit_command(selected_path, closest_index)
        
        msg = Float32MultiArray()
        msg.data = [steering_angle]
        self.gap_pub.publish(msg)
        
        self.publish_path_viz(selected_path)
        self.publish_all_paths(data, selected_path, ranges)

    def check_path_validity(self, path, scan_data, ranges, closest_idx):
        check_dist = 1.0 
        dist_checked = 0.0
        curr_idx = closest_idx
        
        while dist_checked < check_dist:
            pt_global = path[curr_idx]
            
            dx = pt_global[0] - self.x
            dy = pt_global[1] - self.y
            
            x_local = dx * math.cos(-self.heading) - dy * math.sin(-self.heading)
            y_local = dx * math.sin(-self.heading) + dy * math.cos(-self.heading)
            
            if x_local < 0:
                curr_idx = (curr_idx + 1) % len(path)
                continue
                
            r = math.sqrt(x_local**2 + y_local**2)
            theta = math.atan2(y_local, x_local)
            
            if scan_data.angle_min <= theta <= scan_data.angle_max:
                idx = int((theta - scan_data.angle_min) / scan_data.angle_increment)
                if 0 <= idx < len(ranges):
                    lidar_dist = ranges[idx]
                    
                    obs_x = lidar_dist * math.cos(theta)
                    obs_y = lidar_dist * math.sin(theta)
                    dist_to_obs = math.sqrt((obs_x - x_local)**2 + (obs_y - y_local)**2)
                    
                    if dist_to_obs < 0.35: 
                        return False
            
            step_dist = np.linalg.norm(path[(curr_idx+1)%len(path), :2] - path[curr_idx, :2])
            dist_checked += step_dist * 2.0 
            curr_idx = (curr_idx + 2) % len(path)
            
        return True

    # --- UPDATED: Takes closest_idx as argument ---
    def get_pure_pursuit_command(self, path, closest_idx):
        # REMOVED: np.argmin global search
        # We trust the 'closest_idx' passed from scan_callback
        
        # 1. Find lookahead point
        target_idx = closest_idx
        
        # Look forward, handling wrap-around
        for i in range(closest_idx, len(path) + closest_idx): 
            idx = i % len(path)
            dist = np.linalg.norm(path[idx, :2] - np.array([self.x, self.y]))
            if dist > LOOKAHEAD_DIST:
                target_idx = idx
                break
        
        target_pt = path[target_idx]
        
        # 2. Visualize Goal (Helps debug why it turns left)
        marker = Marker()
        marker.header.frame_id = "map"; marker.type = Marker.SPHERE; marker.action = Marker.ADD
        marker.pose.position.x = target_pt[0]; marker.pose.position.y = target_pt[1]
        marker.scale.x = 0.3; marker.scale.y = 0.3; marker.scale.z = 0.3
        marker.color.a = 1.0; marker.color.r = 1.0; marker.color.g = 1.0 # White Sphere
        self.goal_pub.publish(marker)
        
        # 3. Calculate Steering
        dx = target_pt[0] - self.x
        dy = target_pt[1] - self.y
        
        x_rel = dx * math.cos(self.heading) + dy * math.sin(self.heading)
        y_rel = -dx * math.sin(self.heading) + dy * math.cos(self.heading)
        
        alpha = math.atan2(y_rel, x_rel)
        actual_lookahead = math.sqrt(dx**2 + dy**2)
        
        steering_angle = math.atan2(2.0 * WHEELBASE_LEN * math.sin(alpha), actual_lookahead)
        
        return steering_angle 

    def publish_path_viz(self, path):
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

    def publish_all_paths(self, scan_data, selected_path, ranges):
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
            marker.scale.x = 0.05 

            is_valid = self.check_path_validity(path, scan_data, ranges, self.last_closest_index)

            if np.array_equal(path, selected_path):
                marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0; marker.color.a = 1.0
                marker.scale.x = 0.1
            elif is_valid:
                marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0; marker.color.a = 0.5
            else:
                marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; marker.color.a = 0.5

            dists = np.linalg.norm(path[:, :2] - np.array([self.x, self.y]), axis=1)
            closest_idx = np.argmin(dists)
            start_viz = max(0, closest_idx - 50)
            end_viz = min(len(path), closest_idx + 150)
            
            for pt in path[start_viz:end_viz:5]:
                p = Point()
                p.x = pt[0]; p.y = pt[1]; p.z = 0.0
                marker.points.append(p)

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

if __name__ == "__main__":
    try:
        DistFinder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass