#!/usr/bin/env python
import rospy
import sys
import math
import tf
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PolygonStamped, Point32, PoseStamped
from nav_msgs.msg import Path

# --- TUNING CONFIGURATION ---
STEERING_RANGE = 100.0           # Servo range [-100, 100]
WHEELBASE_LEN = 0.325            # Vehicle wheelbase
MAX_STEERING_ANGLE = 0.4189      # ~24 degrees (Physical Limit)

# --- SPEED SETTINGS (Steering Angle Based) ---
MAX_SPEED = 50.0                 # Max speed on straights
MIN_SPEED = 10.0                 # Minimum speed to maintain momentum in corners

# Cornering Aggression: 0.0 = No slowdown, 1.0 = Stop completely at full turn
# 0.5 means: At full steering lock, drop speed by 50% (to 25.0)
CORNERING_AGGRESSION = 0.5       

# --- LOOKAHEAD SETTINGS (Velocity Based) ---
# Higher speeds require looking further ahead to prevent oscillation
LOOKAHEAD_GAIN = 0.4             # Lookahead = Velocity * Gain
MIN_LOOKAHEAD = 1.5              # Minimum lookahead distance
MAX_LOOKAHEAD = 4.0              # Cap lookahead so we don't cut corners too much

# --- GLOBALS ---
plan = []
car_name = str(sys.argv[1])
trajectory_name = str(sys.argv[2]) # Kept for arg compatibility

wp_seq = 0
last_closest_index = 0 

# Publishers
command_pub = rospy.Publisher('/{}/offboard/command'.format(car_name), AckermannDrive, queue_size=1)
polygon_pub = rospy.Publisher('/{}/purepursuit_control/visualize'.format(car_name), PolygonStamped, queue_size=1)

def dynamic_path_callback(msg):
    global plan
    new_plan = []
    # Convert Path msg back to list of [x, y]
    for pose in msg.poses:
        new_plan.append([pose.pose.position.x, pose.pose.position.y])
        
    if len(new_plan) > 0:
        plan = new_plan

def purepursuit_control_node(data):
    global wp_seq, plan, last_closest_index

    if not plan:
        return 

    # 1. Car Position & Heading
    odom_x = data.pose.position.x
    odom_y = data.pose.position.y
    
    quaternion = (data.pose.orientation.x, data.pose.orientation.y, 
                  data.pose.orientation.z, data.pose.orientation.w)
    heading = tf.transformations.euler_from_quaternion(quaternion)[2]

    # 2. Find Closest Point (Windowed Search + Circular Logic)
    min_dist_sq = float('inf')
    closest_index = last_closest_index
    
    # Search Window: Check -10 to +50 points around last known index
    start_search = last_closest_index - 10
    end_search = last_closest_index + 50
    
    for i in range(start_search, end_search):
        idx = i % len(plan) # Handle Wrap-Around (End -> Start)
        dx = odom_x - plan[idx][0]
        dy = odom_y - plan[idx][1]
        dist_sq = dx*dx + dy*dy
        if dist_sq < min_dist_sq:
            min_dist_sq = dist_sq
            closest_index = idx

    # Fail-safe: If lost (> 5 meters away), do Global Search
    if min_dist_sq > 25.0: 
        min_dist_sq = float('inf')
        for i in range(len(plan)):
            dx = odom_x - plan[i][0]
            dy = odom_y - plan[i][1]
            dist_sq = dx*dx + dy*dy
            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                closest_index = i

    last_closest_index = closest_index # Update state
    
    pose_x = plan[closest_index][0]
    pose_y = plan[closest_index][1]
    
    # 3. Dynamic Lookahead Calculation (For Steering Stability)
    # Get current velocity (if available, else assume slow)
    current_vel = data.twist.linear.x if hasattr(data, 'twist') else 5.0
    
    calculated_lookahead = current_vel * LOOKAHEAD_GAIN
    lookahead = max(MIN_LOOKAHEAD, min(MAX_LOOKAHEAD, calculated_lookahead))

    # 4. Find Goal Point (Circular Search)
    target_index = closest_index
    
    for i in range(closest_index, closest_index + len(plan)):
        idx = i % len(plan)
        dist = math.sqrt((plan[idx][0] - odom_x)**2 + (plan[idx][1] - odom_y)**2)
        if dist > lookahead:
            target_index = idx
            break
            
    target_pt = plan[target_index]
    
    # 5. Calculate Steering
    dx = target_pt[0] - odom_x
    dy = target_pt[1] - odom_y
    
    x_rel = dx * math.cos(heading) + dy * math.sin(heading)
    y_rel = -dx * math.sin(heading) + dy * math.cos(heading)
    
    alpha = math.atan2(y_rel, x_rel)
    actual_lookahead = math.sqrt(dx**2 + dy**2)
    
    steering_angle_rad = math.atan2(2.0 * WHEELBASE_LEN * math.sin(alpha), actual_lookahead)
    
    # 6. Clamp Steering
    steering_angle_rad = max(-MAX_STEERING_ANGLE, min(MAX_STEERING_ANGLE, steering_angle_rad))
    
    # 7. Create Command
    command = AckermannDrive()
    command.steering_angle = (steering_angle_rad / MAX_STEERING_ANGLE) * STEERING_RANGE
    
    # 8. VELOCITY SCALING (Based on Steering Angle)
    current_steer = abs(steering_angle_rad)
    
    # Ignore tiny jitters (less than ~3 degrees) for max speed
    if current_steer > 0.05: 
        # Calculate ratio: 0.0 (straight) to 1.0 (full turn)
        steer_ratio = current_steer / MAX_STEERING_ANGLE
        
        # Apply drop factor
        # e.g. If Aggression is 0.5 and Ratio is 1.0 (full turn) -> Drop speed by 50%
        speed_drop_factor = steer_ratio * CORNERING_AGGRESSION
        
        target_speed = MAX_SPEED * (1.0 - speed_drop_factor)
    else:
        target_speed = MAX_SPEED

    # Ensure we don't drop below minimum speed
    command.speed = max(MIN_SPEED, target_speed)

    command_pub.publish(command)
    
    # 9. Visualization
    poly = PolygonStamped()
    poly.header.frame_id = "map"
    poly.header.stamp = rospy.Time.now()
    p1 = Point32(x=odom_x, y=odom_y)
    p2 = Point32(x=pose_x, y=pose_y)
    p3 = Point32(x=target_pt[0], y=target_pt[1])
    poly.polygon.points = [p2, p1, p3]
    poly.header.seq = wp_seq
    wp_seq += 1
    polygon_pub.publish(poly)

if __name__ == '__main__':
    try:
        rospy.init_node('pure_pursuit', anonymous=True)
        rospy.loginfo('Waiting for dynamic trajectory from dist_finder...')
        
        rospy.Subscriber('/{}/particle_filter/viz/inferred_pose'.format(car_name), PoseStamped, purepursuit_control_node)
        rospy.Subscriber('/{}/selected_path'.format(car_name), Path, dynamic_path_callback)
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass