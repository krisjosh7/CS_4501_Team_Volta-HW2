#!/usr/bin/env python
import rospy
import sys
import math
import tf
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PolygonStamped, Point32, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


# --- CONFIGURATION ---
STEERING_RANGE = 100.0
WHEELBASE_LEN = 0.325
LOOKAHEAD_DIST = 1.5
MAX_STEERING_ANGLE = math.radians(30) # ~0.52 rad

# --- GLOBALS ---
plan = []
car_name = str(sys.argv[1])
# We don't strictly need trajectory_name arg since we listen to /selected_path, 
# but we keep it to not break rosrun args.
trajectory_name = str(sys.argv[2]) 

wp_seq = 0
last_closest_index = 0 # State for windowed search

# Publishers
command_pub = rospy.Publisher('/{}/offboard/command'.format(car_name), AckermannDrive, queue_size=1)
polygon_pub = rospy.Publisher('/{}/purepursuit_control/visualize'.format(car_name), PolygonStamped, queue_size=1)
# We don't need to publish path again since dist_finder does it, but we can if debugging is needed.
target_marker = rospy.Publisher('/car_8/target_marker', Marker, queue_size=1)
steering_pub = rospy.Publisher("/car_8/steering_arrow", Marker, queue_size=2)

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

    # 1. Car Position
    odom_x = data.pose.position.x
    odom_y = data.pose.position.y
    
    # Heading (Yaw)
    quaternion = (data.pose.orientation.x, data.pose.orientation.y, 
                  data.pose.orientation.z, data.pose.orientation.w)
    heading = tf.transformations.euler_from_quaternion(quaternion)[2]

    # 2. Find Closest Point (Windowed Search)
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
    
    # 3. Dynamic Lookahead (Optional Tuning)
    # current_vel = data.twist.linear.x if hasattr(data, 'twist') else 0.0
    # lookahead = max(1.0, min(2.5, current_vel * 0.2))
    lookahead = LOOKAHEAD_DIST

    # 4. Find Goal Point (FIXED: Handles Wrap-Around)
    target_index = closest_index
    
    # Loop continuously through the array indices using Modulo (%) 
    # This prevents stopping at the end of the array when the goal is actually at the beginning.
    for i in range(closest_index, closest_index + len(plan)):
        idx = i % len(plan)
        dist = math.sqrt((plan[idx][0] - odom_x)**2 + (plan[idx][1] - odom_y)**2)
        if dist > lookahead:
            target_index = idx
            break
            
    target_pt = plan[target_index]

    marker = Marker()
    marker.header = data.header
    marker.ns = "target"
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD

    marker.pose.position.x = target_pt[0]
    marker.pose.position.y = target_pt[1]
    marker.pose.position.z = 0.0

    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2

    marker.color.r = 0.0
    marker.color.a = 1.0
    marker.color.b = 1.0
    marker.color.g = 0.0
    target_marker.publish(marker)
    
    # 5. Calculate Steering
    dx = target_pt[0] - odom_x
    dy = target_pt[1] - odom_y
    
    x_rel = dx * math.cos(heading) + dy * math.sin(heading)
    y_rel = -dx * math.sin(heading) + dy * math.cos(heading)
 
    ratio = y_rel/LOOKAHEAD_DIST
    ratio = max(-1.0, min(1.0, ratio))
    
    alpha = math.asin(ratio)
    actual_lookahead = math.sqrt(dx**2 + dy**2)
    
    steering_angle_rad = math.atan2(2.0 * WHEELBASE_LEN * math.sin(alpha), LOOKAHEAD_DIST)
    
    # 6. Clamp and Publish
    steering_angle_rad = max(-MAX_STEERING_ANGLE, min(MAX_STEERING_ANGLE, steering_angle_rad))
    

    steering = Marker()
    steering.header.frame_id = "car_8_base_link"  # C_8_basehanged to car_8
    steering.header.stamp = rospy.Time.now()
    steering.type = Marker.ARROW
    steering.id = 0
    steering.scale.x = 0.1  # arrow length
    steering.scale.y = 0.1  # arrow width
    steering.scale.z = 0.1  # arrow heighter
    steering.pose.orientation.w = 1.0
    steering.pose.position.x = 0.0
    steering.pose.position.y = 0.0
    steering.pose.position.z = 0.0


    q = tf.transformations.quaternion_from_euler(0, 0, steering_angle_rad)
    steering.pose.orientation.x = q[0]
    steering.pose.orientation.y = q[1]
    steering.pose.orientation.z = q[2]
    steering.pose.orientation.w = q[3]

    steering.color.r = 0.0
    steering.color.g = 0.0
    steering.color.b = 1.0
    steering.color.a = 1.0
    
    # Arrow points from origin to target direction
    steering.points = [
        Point(0, 0, 0),
        Point(
            math.cos(steering_angle_rad),
            math.sin(steering_angle_rad),
            0
        )
    ]
    
    steering_pub.publish(steering)

    command = AckermannDrive()
    command.steering_angle = (steering_angle_rad / MAX_STEERING_ANGLE) * STEERING_RANGE
   

    # Dynamic Velocity
    steering_difficulty = abs(steering_angle_rad)
    if steering_difficulty < math.radians(10):
        command.speed = 45.0 
    elif steering_difficulty < math.radians(20):
        command.speed = 35.0
    else:
        command.speed = 20.0 

    command_pub.publish(command)
    
    # Visualization
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