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

# --- TUNING PARAMETERS ---
LOOKAHEAD_MIN = 0.5   # Minimum lookahead (meters) for low speeds
LOOKAHEAD_MAX = 2.5   # Maximum lookahead (meters) for high speeds
LOOKAHEAD_GAIN = 0.25 # Lookahead = Gain * Velocity
WHEELBASE_LEN = 0.325
STEERING_RANGE = 100.0 # Range [-100, 100] matches your servo setup
MAX_STEERING_ANGLE = 0.4189 # ~24 degrees (Matches your control.py limit)

# --- GLOBAL VARIABLES ---
plan = []               # [x, y, velocity]
frame_id = 'map'
try:
    car_name = str(sys.argv[1])
    trajectory_name = str(sys.argv[2])
except IndexError:
    # Fallback defaults if args aren't provided
    car_name = "car_8"
    trajectory_name = "optimal_raceline"

# State variables
current_velocity = 0.0
wp_seq = 0
control_polygon = PolygonStamped()

# WAYPOINT TRACKING STATE (The Fix)
last_closest_index = 0 

# Publishers
command_pub = rospy.Publisher('/{}/offboard/command'.format(car_name), AckermannDrive, queue_size=1)
polygon_pub = rospy.Publisher('/{}/purepursuit_control/visualize'.format(car_name), PolygonStamped, queue_size=1)
path_pub    = rospy.Publisher('/{}/purepursuit_control/path'.format(car_name), Path, queue_size=1, latch=True)

def construct_path():
    """ 
    Loads the trajectory CSV. 
    Expects format: [x, y, velocity] 
    """
    file_path = os.path.expanduser('~/depend_ws/src/f1tenth_purepursuit/path/{}.csv'.format(trajectory_name))
    
    rospy.loginfo(f"Loading path from: {file_path}")
    
    try:
        with open(file_path) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            for waypoint in csv_reader:
                x = float(waypoint[0])
                y = float(waypoint[1])
                v = float(waypoint[2]) if len(waypoint) > 2 else 1.0
                plan.append([x, y, v])
    except Exception as e:
        rospy.logerr(f"Error loading CSV: {e}")
        return

    # Create and publish the path for visualization (Rviz)
    path_msg = Path()
    path_msg.header.frame_id = frame_id
    path_msg.header.stamp = rospy.Time.now()
    
    for waypoint in plan:
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = path_msg.header.stamp
        pose.pose.position.x = waypoint[0]
        pose.pose.position.y = waypoint[1]
        pose.pose.orientation.w = 1.0
        path_msg.poses.append(pose)
    
    path_pub.publish(path_msg)
    rospy.loginfo(f"Path constructed with {len(plan)} waypoints.")

def odom_callback(data):
    """
    Updates the current velocity of the car.
    """
    global current_velocity
    current_velocity = data.twist.twist.linear.x

def purepursuit_control_node(data):
    global wp_seq
    global current_velocity
    global last_closest_index # Access the tracking variable
    
    if not plan:
        return

    command = AckermannDrive()
    
    # 1. Obtain current position
    odom_x = data.pose.position.x
    odom_y = data.pose.position.y
    
    # Calculate heading angle (yaw)
    heading = tf.transformations.euler_from_quaternion((
        data.pose.orientation.x,
        data.pose.orientation.y,
        data.pose.orientation.z,
        data.pose.orientation.w
    ))[2]

    # --- STEP 2: FIND CLOSEST POINT (WINDOWED SEARCH FIX) ---
    
    min_dist_sq = float('inf')
    closest_index = last_closest_index
    
    # Search Window: Look 10 points behind and 50 points ahead of previous location.
    # This prevents jumping to the wrong part of the track.
    search_start = last_closest_index - 10
    search_end = last_closest_index + 50
    
    for i in range(search_start, search_end):
        idx = i % len(plan) # Handle wrap-around (Start/Finish line)
        
        dx = odom_x - plan[idx][0]
        dy = odom_y - plan[idx][1]
        dist_sq = dx*dx + dy*dy
        
        if dist_sq < min_dist_sq:
            min_dist_sq = dist_sq
            closest_index = idx

    # SAFETY CHECK: If we are "lost" (closest point is too far), do a global search
    # This handles manual resets in simulation or startup.
    if min_dist_sq > 10.0: # If closest point is > 3.16m away
        rospy.logwarn("Waypoint lost! Performing global search reset.")
        min_dist_sq = float('inf')
        for i in range(len(plan)):
            dx = odom_x - plan[i][0]
            dy = odom_y - plan[i][1]
            dist_sq = dx*dx + dy*dy
            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                closest_index = i
    
    # Update state for next iteration
    last_closest_index = closest_index
    
    pose_x = plan[closest_index][0]
    pose_y = plan[closest_index][1]

    # --- STEP 3: DYNAMIC LOOKAHEAD ---
    lookahead_distance = LOOKAHEAD_GAIN * current_velocity
    lookahead_distance = max(LOOKAHEAD_MIN, min(lookahead_distance, LOOKAHEAD_MAX))

    # --- STEP 4: FIND GOAL POINT ---
    # Start searching forward from the closest index
    target_index = closest_index
    found_target = False
    
    for i in range(closest_index, closest_index + len(plan)):
        idx = i % len(plan)
        dist = math.sqrt((plan[idx][0] - odom_x)**2 + (plan[idx][1] - odom_y)**2)
        if dist > lookahead_distance:
            target_index = idx
            found_target = True
            break
            
    # If we didn't find a point far enough (e.g. end of track without loop), take the last one
    if not found_target:
        target_index = (closest_index + 5) % len(plan) # Just look a bit ahead
    
    target_x = plan[target_index][0]
    target_y = plan[target_index][1]
    target_v = plan[target_index][2]

    # --- STEP 5: CALCULATE STEERING ---
    # Transform target to vehicle frame
    dx = target_x - odom_x
    dy = target_y - odom_y
    
    x_rel = dx * math.cos(heading) + dy * math.sin(heading)
    y_rel = -dx * math.sin(heading) + dy * math.cos(heading)

    alpha = math.atan2(y_rel, x_rel)
    actual_lookahead_dist = math.sqrt(dx*dx + dy*dy)
    
    # Pure Pursuit Formula
    steering_angle_rad = math.atan2(2.0 * WHEELBASE_LEN * math.sin(alpha), actual_lookahead_dist)

    # Clamp and Scale
    steering_angle_rad = max(-MAX_STEERING_ANGLE, min(MAX_STEERING_ANGLE, steering_angle_rad))
    command.steering_angle = (steering_angle_rad / MAX_STEERING_ANGLE) * STEERING_RANGE

    # Assign Velocity

    max_speed = 30.0  # m/s, tune this
    min_speed = 10.0  # m/s, tune this
    
    # Linearly scale speed based on the magnitude of the steering angle
    abs_steering = abs(command.steering_angle)
    command.speed = max_speed - (abs_steering / STEERING_RANGE) * (max_speed - min_speed)

    # command.speed = target_v

    command_pub.publish(command)

    # --- VISUALIZATION ---
    base_link = Point32()
    nearest_pose = Point32()
    nearest_goal = Point32()
    
    base_link.x = odom_x
    base_link.y = odom_y
    nearest_pose.x = pose_x
    nearest_pose.y = pose_y
    nearest_goal.x = target_x
    nearest_goal.y = target_y
    
    control_polygon.header.frame_id = frame_id
    control_polygon.polygon.points = [nearest_pose, base_link, nearest_goal]
    control_polygon.header.seq = wp_seq
    control_polygon.header.stamp = rospy.Time.now()
    wp_seq += 1
    polygon_pub.publish(control_polygon)

if __name__ == '__main__':
    try:
        rospy.init_node('pure_pursuit', anonymous=True)
        
        if not plan:
            rospy.loginfo('Obtaining trajectory...')
            construct_path()

        # Subscriber for Pose (Localization)
        rospy.Subscriber('/{}/particle_filter/viz/inferred_pose'.format(car_name), PoseStamped, purepursuit_control_node)
        
        # Subscriber for Velocity (Odometry)
        rospy.Subscriber('/{}/odom'.format(car_name), Odometry, odom_callback)
        
        rospy.spin()

    except rospy.ROSInterruptException:
        pass