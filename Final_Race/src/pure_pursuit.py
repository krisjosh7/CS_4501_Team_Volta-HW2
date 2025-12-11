#!/usr/bin/env python
import rospy
import os
import sys
import csv
import math
import tf
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PolygonStamped, Point32, PoseStamped
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker 

# --- TUNING PARAMETERS (ADJUSTED FOR NON-METRIC SPEED) ---
# Keep these in METERS because your map (CSV) is in meters
LOOKAHEAD_MIN = 0.8   
LOOKAHEAD_MAX = 2.5   # Cap this! Don't let it look 20m ahead.

# CRITICAL: Since speed is ~45, Gain must be small to get a result in Meters.
# 0.04 * 50 (speed) = 2.0 meters lookahead
LOOKAHEAD_GAIN = 0.04 

WHEELBASE_LEN = 0.325
STEERING_RANGE = 100.0 # Output range -100 to 100
MAX_STEERING_ANGLE = 0.4189 # Internal physics limit (radians)

# --- SPEED PARAMETERS (UNITLESS / PERCENTAGE) ---
# If 45 is average:
STRAIGHT_SPEED = 65.0  # Max throttle on straights
CORNER_SPEED   = 25.0  # Brake speed for sharp corners

# --- GLOBAL VARIABLES ---
plan = []               
frame_id = 'map'
try:
    car_name = str(sys.argv[1])
    trajectory_name = str(sys.argv[2])
except IndexError:
    car_name = "car_8"
    trajectory_name = "base_map_3_raceline"

# State variables
current_velocity = 0.0
wp_seq = 0
control_polygon = PolygonStamped()
last_closest_index = 0 

# Publishers
command_pub = rospy.Publisher('/{}/offboard/command'.format(car_name), AckermannDrive, queue_size=1)
polygon_pub = rospy.Publisher('/{}/purepursuit_control/visualize'.format(car_name), PolygonStamped, queue_size=1)
marker_pub  = rospy.Publisher('/{}/purepursuit_control/target_marker'.format(car_name), Marker, queue_size=1)
path_pub    = rospy.Publisher('/{}/purepursuit_control/path'.format(car_name), Path, queue_size=1, latch=True)

def construct_path():
    home = os.path.expanduser('~')
    possible_paths = [
        f"{home}/depend_ws/src/f1tenth_purepursuit/path/{trajectory_name}.csv",
        f"{home}/catkin_ws/src/f1tenth_purepursuit/path/{trajectory_name}.csv",
        f"{trajectory_name}.csv"
    ]
    
    file_path = ""
    for p in possible_paths:
        if os.path.exists(p):
            file_path = p
            break
            
    if file_path == "":
        rospy.logerr(f"CSV file not found. Checked: {possible_paths}")
        return

    rospy.loginfo(f"Loading path from: {file_path}")
    
    try:
        with open(file_path) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            for waypoint in csv_reader:
                plan.append([float(waypoint[0]), float(waypoint[1])])
    except Exception as e:
        rospy.logerr(f"Error loading CSV: {e}")
        return

    path_msg = Path()
    path_msg.header.frame_id = frame_id
    path_msg.header.stamp = rospy.Time.now()
    for pt in plan:
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = pt[0]
        pose.pose.position.y = pt[1]
        pose.pose.orientation.w = 1.0
        path_msg.poses.append(pose)
    path_pub.publish(path_msg)

def odom_callback(data):
    global current_velocity
    current_velocity = data.twist.twist.linear.x

def purepursuit_control_node(data):
    global wp_seq, current_velocity, last_closest_index 
    
    if not plan:
        return

    command = AckermannDrive()
    
    odom_x = data.pose.position.x
    odom_y = data.pose.position.y
    
    heading = tf.transformations.euler_from_quaternion((
        data.pose.orientation.x,
        data.pose.orientation.y,
        data.pose.orientation.z,
        data.pose.orientation.w
    ))[2]

    # --- 1. WINDOWED SEARCH ---
    min_dist_sq = float('inf')
    closest_index = last_closest_index
    path_len = len(plan)
    
    for i in range(last_closest_index - 10, last_closest_index + 50):
        idx = i % path_len 
        dx = odom_x - plan[idx][0]
        dy = odom_y - plan[idx][1]
        d_sq = dx*dx + dy*dy
        if d_sq < min_dist_sq:
            min_dist_sq = d_sq
            closest_index = idx

    # Lost reset logic
    if min_dist_sq > 10.0: 
        min_dist_sq = float('inf')
        for i in range(path_len):
            dx = odom_x - plan[i][0]
            dy = odom_y - plan[i][1]
            d_sq = dx*dx + dy*dy
            if d_sq < min_dist_sq:
                min_dist_sq = d_sq
                closest_index = i
    
    last_closest_index = closest_index

    # --- 2. DYNAMIC LOOKAHEAD ---
    # Lookahead = Gain (0.04) * Speed (50) = 2.0 meters
    lookahead = LOOKAHEAD_GAIN * current_velocity
    lookahead = max(LOOKAHEAD_MIN, min(lookahead, LOOKAHEAD_MAX))

    # --- 3. FIND TARGET ---
    target_index = closest_index
    found_target = False
    
    for i in range(closest_index, closest_index + path_len):
        idx = i % path_len
        dist = math.sqrt((plan[idx][0] - odom_x)**2 + (plan[idx][1] - odom_y)**2)
        if dist > lookahead:
            target_index = idx
            found_target = True
            break
            
    if not found_target:
        target_index = (closest_index + 5) % path_len
    
    target_x = plan[target_index][0]
    target_y = plan[target_index][1]

    # --- 4. CALCULATE STEERING ---
    dx = target_x - odom_x
    dy = target_y - odom_y
    
    x_rel = dx * math.cos(heading) + dy * math.sin(heading)
    y_rel = -dx * math.sin(heading) + dy * math.cos(heading)

    alpha = math.atan2(y_rel, x_rel)
    L_dist = math.sqrt(dx*dx + dy*dy)
    
    steering_angle_rad = math.atan2(2.0 * WHEELBASE_LEN * math.sin(alpha), L_dist)

    # Clamp radians
    steering_angle_rad = max(-MAX_STEERING_ANGLE, min(MAX_STEERING_ANGLE, steering_angle_rad))
    
    # Scale to [-100, 100]
    command.steering_angle = (steering_angle_rad / MAX_STEERING_ANGLE) * STEERING_RANGE

    # --- 5. DYNAMIC VELOCITY (The 'Average 45' Fix) ---
    
    # Calculate steering intensity (0.0 to 1.0)
    turn_intensity = abs(command.steering_angle) / STEERING_RANGE
    
    if turn_intensity < 0.15: 
        # Straight line: Go fast
        command.speed = STRAIGHT_SPEED
    elif turn_intensity < 0.35:
        # Shallow turn: Average speed
        command.speed = (STRAIGHT_SPEED + CORNER_SPEED) / 2 # Approx 45
    else:
        # Sharp turn: Slow down
        command.speed = CORNER_SPEED
        
    command_pub.publish(command)

    # --- VISUALIZATION ---
    control_polygon.header.frame_id = frame_id
    control_polygon.header.stamp = rospy.Time.now()
    control_polygon.polygon.points = [
        Point32(x=plan[closest_index][0], y=plan[closest_index][1], z=0),
        Point32(x=odom_x, y=odom_y, z=0),
        Point32(x=target_x, y=target_y, z=0)
    ]
    polygon_pub.publish(control_polygon)
    
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = target_x
    marker.pose.position.y = target_y
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.4; marker.scale.y = 0.4; marker.scale.z = 0.4
    marker.color.a = 1.0; marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0
    marker_pub.publish(marker)

if __name__ == '__main__':
    try:
        rospy.init_node('pure_pursuit', anonymous=True)
        if not plan:
            construct_path()
        rospy.Subscriber('/{}/particle_filter/viz/inferred_pose'.format(car_name), PoseStamped, purepursuit_control_node)
        rospy.Subscriber('/{}/odom'.format(car_name), Odometry, odom_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass