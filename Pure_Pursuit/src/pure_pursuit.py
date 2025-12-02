import rospy
import os
import sys
import csv
import math
import tf
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PolygonStamped, Point32, PoseStamped
from nav_msgs.msg import Path, Odometry # Added Odometry for velocity feedback

# --- TUNING PARAMETERS ---
LOOKAHEAD_MIN = 0.5  # Minimum lookahead (meters) for low speeds
LOOKAHEAD_MAX = 2.5  # Maximum lookahead (meters) for high speeds
LOOKAHEAD_GAIN = 0.25 # Lookahead = Gain * Velocity
WHEELBASE_LEN = 0.325
STEERING_RANGE = 100.0 # Range [-100, 100] matches your servo setup
MAX_STEERING_ANGLE = math.radians(30) # Physical limit

# --- GLOBAL VARIABLES ---
plan = []               # [x, y, velocity]
path_resolution = []
frame_id = 'map'
car_name = str(sys.argv[1])
trajectory_name = str(sys.argv[2])

# State variables
current_velocity = 0.0
wp_seq = 0
control_polygon = PolygonStamped()

# Publishers
command_pub = rospy.Publisher('/{}/offboard/command'.format(car_name), AckermannDrive, queue_size=1)
polygon_pub = rospy.Publisher('/{}/purepursuit_control/visualize'.format(car_name), PolygonStamped, queue_size=1)
path_pub    = rospy.Publisher('/{}/purepursuit_control/path'.format(car_name), Path, queue_size=1, latch=True)

def construct_path():
    """ 
    Loads the trajectory CSV. 
    Expects format: [x, y, velocity] 
    """
    # Use the script's directory to find the csv file or specific path
    # Ensure this matches where you saved the optimized raceline
    file_path = os.path.expanduser('~/depend_ws/src/f1tenth_purepursuit/path/{}.csv'.format(trajectory_name))
    
    rospy.loginfo(f"Loading path from: {file_path}")
    
    try:
        with open(file_path) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            for waypoint in csv_reader:
                # Store x, y, and velocity (default to 1.0 if v is missing)
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
    # Main control function for pure pursuit algorithm
    global wp_seq
    global current_velocity
    
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

    # 2. Find the Base Projection (Closest Point)
    min_dist_sq = float('inf')
    closest_index = 0
    
    # Optimization: Search locally around the last known closest point (optional, doing full search for safety)
    for i in range(len(plan)):
        dx = odom_x - plan[i][0]
        dy = odom_y - plan[i][1]
        dist_sq = dx*dx + dy*dy
        if dist_sq < min_dist_sq:
            min_dist_sq = dist_sq
            closest_index = i
            
    pose_x = plan[closest_index][0]
    pose_y = plan[closest_index][1]

    # 3. Dynamic Lookahead Distance
    # Lookahead scales with speed. 
    # Fast = Look far (smoother). Slow = Look close (cut corners).
    lookahead_distance = LOOKAHEAD_GAIN * current_velocity
    lookahead_distance = max(LOOKAHEAD_MIN, min(lookahead_distance, LOOKAHEAD_MAX))

    # 4. Find Goal Point
    target_index = closest_index
    for i in range(closest_index, len(plan)):
        dist = math.sqrt((plan[i][0] - odom_x)**2 + (plan[i][1] - odom_y)**2)
        if dist > lookahead_distance:
            target_index = i
            break
            
    # Handle end of track (looping or stopping)
    if target_index == closest_index and closest_index == len(plan) - 1:
        target_index = 0 # Loop back to start
    
    target_x = plan[target_index][0]
    target_y = plan[target_index][1]
    target_v = plan[target_index][2] # Get optimal velocity from CSV

    # 5. Calculate Steering Angle
    # Transform target to vehicle frame
    dx = target_x - odom_x
    dy = target_y - odom_y
    
    x_rel = dx * math.cos(heading) + dy * math.sin(heading)
    y_rel = -dx * math.sin(heading) + dy * math.cos(heading)

    alpha = math.atan2(y_rel, x_rel)
    actual_lookahead_dist = math.sqrt(dx*dx + dy*dy)
    
    # Pure Pursuit Formula: delta = arctan( 2L sin(alpha) / Ld )
    steering_angle_rad = math.atan2(2.0 * WHEELBASE_LEN * math.sin(alpha), actual_lookahead_dist)

    # 6. Clamp and Scale Steering
    steering_angle_rad = max(-MAX_STEERING_ANGLE, min(MAX_STEERING_ANGLE, steering_angle_rad))
    command.steering_angle = (steering_angle_rad / MAX_STEERING_ANGLE) * STEERING_RANGE

    # 7. Assign Velocity
    # Use the optimized velocity directly from the CSV
    command.speed = target_v

    command_pub.publish(command)

    # 8. Visualization
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
        
        # Subscriber for Velocity (Odometry) - CRITICAL for dynamic lookahead
        rospy.Subscriber('/{}/odom'.format(car_name), Odometry, odom_callback)
        
        rospy.spin()

    except rospy.ROSInterruptException:
        pass