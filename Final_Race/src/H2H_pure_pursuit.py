#!/usr/bin/env python

# Import necessary libraries
import rospy
import os
import sys
import csv
import math
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path # For path visualization
import tf

# Global variables for storing the path, path resolution, frame ID, and car details
plan                = []
path_resolution     = []
frame_id            = 'map'
car_name            = str(sys.argv[1])
trajectory_name     = str(sys.argv[2])

# Publishers for sending driving commands and visualizing the control polygon
command_pub         = rospy.Publisher('/{}/offboard/command'.format(car_name), AckermannDrive, queue_size = 1)
polygon_pub         = rospy.Publisher('/{}/purepursuit_control/visualize'.format(car_name), PolygonStamped, queue_size = 1)
path_pub            = rospy.Publisher('/{}/purepursuit_control/path'.format(car_name), Path, queue_size=1, latch=True)

# Global variables for waypoint sequence and current polygon
global wp_seq
global curr_polygon

wp_seq          = 0
control_polygon = PolygonStamped()

def dynamic_path_callback(msg):
    global plan
    new_plan = []
    # Convert Path msg back to list of [x, y]
    # NOTE: The incoming path does NOT have velocity. We just get x, y.
    for pose in msg.poses:
        x = pose.pose.position.x
        y = pose.pose.position.y
        new_plan.append([x, y])
        
    if len(new_plan) > 0:
        plan = new_plan

def construct_path():
    # Placeholder for static path loading if needed, but primarily we wait for dynamic path
    pass

# Steering Range from -100.0 to 100.0
STEERING_RANGE = 100.0

# vehicle physical parameters
WHEELBASE_LEN       = 0.325

def purepursuit_control_node(data):
    # Main control function for pure pursuit algorithm

    # Create an empty ackermann drive message that we will populate later with the desired steering angle and speed.
    command = AckermannDrive()

    global wp_seq
    global curr_polygon
    global plan

    if not plan:
        return # modifying safety check

    # Obtain the current position of the race car from the inferred_pose message
    odom_x = data.pose.position.x
    odom_y = data.pose.position.y


    # The reference path is stored in the 'plan' array.
    # Find the base projection of the car on this reference path.
    
    min_dist_sq = float('inf')
    closest_index = 0
    for i in range(len(plan)):
        dx = odom_x - plan[i][0]
        dy = odom_y - plan[i][1]
        dist_sq = dx*dx + dy*dy
        if dist_sq < min_dist_sq:
            min_dist_sq = dist_sq
            closest_index = i
    
    pose_x = plan[closest_index][0]
    pose_y = plan[closest_index][1]

    
    # Calculate heading angle of the car (in radians)
    heading = tf.transformations.euler_from_quaternion((data.pose.orientation.x,
                                                        data.pose.orientation.y,
                                                        data.pose.orientation.z,
                                                        data.pose.orientation.w))[2]
    

    # Retrieve current velocity
    current_vel = data.twist.linear.x if hasattr(data, 'twist') else 0.0

    # Scale lookahead: Small at low speed (0.8m), Large at high speed (2.5m)
    # These constants (0.1 scaling factor) need tuning on the track
    lookahead_distance = 0.5 + (0.0175 * current_vel)
    lookahead_distance = max(0.5, min(lookahead_distance, 1.5)) # Clamp values


    # Identify the goal or target point for the car.
    target_index = closest_index
    # Search for the goal point starting from the closest point
    for i in range(closest_index, len(plan)):
        dist = math.sqrt((plan[i][0] - odom_x)**2 + (plan[i][1] - odom_y)**2)
        if dist > lookahead_distance:
            target_index = i
            break
    
    # If we are at the end of the path, the target is the last point
    if target_index == closest_index:
        target_index = len(plan) - 1

    target_x = plan[target_index][0]
    target_y = plan[target_index][1]


    # Implement the pure pursuit algorithm to compute the steering angle
    
    # Transform the target point to the vehicle's coordinate frame
    dx = target_x - odom_x
    dy = target_y - odom_y
    
    # Rotate the vector to the target point by -heading to get it in the car's frame
    x_rel = dx * math.cos(heading) + dy * math.sin(heading)
    y_rel = -dx * math.sin(heading) + dy * math.cos(heading)

    # Calculate the steering angle using the pure pursuit formula.
    alpha = math.atan2(y_rel, x_rel)
    
    actual_lookahead_dist = math.sqrt(dx*dx + dy*dy)
    steering_angle_rad = math.atan2(2.0 * WHEELBASE_LEN * math.sin(alpha), actual_lookahead_dist)


    # Ensure that the calculated steering angle is within the STEERING_RANGE
    
    # Assume a maximum physical steering angle (e.g., 30 degrees)
    max_steering_rad = math.radians(30) 
    
    # Clamp the steering angle to the physical limits of the car
    steering_angle_rad = max(-max_steering_rad, min(max_steering_rad, steering_angle_rad))
    
    # Scale the radian value to the specified STEERING_RANGE [-100, 100]
    command.steering_angle = (steering_angle_rad / max_steering_rad) * STEERING_RANGE
    

    # Implement Dynamic Velocity Scaling instead of a constant speed
    # Since we don't have velocity in the CSV/Path, we calculate it based on steering angle
    
    # Heuristic: Steer hard -> Go slow. Steer straight -> Go fast.
    steering_difficulty = abs(steering_angle_rad) 

    if steering_difficulty < math.radians(10):
        command.speed = 35.0 # Conservative fast speed
    elif steering_difficulty < math.radians(20):
        command.speed = 25.0
    else:
        command.speed = 15.0  # Cornering speed

    command_pub.publish(command)

    # Visualization code
    base_link    = Point32()
    nearest_pose = Point32()
    nearest_goal = Point32()
    base_link.x    = odom_x
    base_link.y    = odom_y
    nearest_pose.x = pose_x
    nearest_pose.y = pose_y
    nearest_goal.x = target_x
    nearest_goal.y = target_y
    control_polygon.header.frame_id = frame_id
    control_polygon.polygon.points  = [nearest_pose, base_link, nearest_goal]
    control_polygon.header.seq      = wp_seq
    control_polygon.header.stamp    = rospy.Time.now()
    wp_seq = wp_seq + 1
    polygon_pub.publish(control_polygon)
    
if __name__ == '__main__':

    try:

        rospy.init_node('pure_pursuit', anonymous = True)
        rospy.loginfo('Waiting for dynamic trajectory from dist_finder...')

        # This node subsribes to the pose estimate provided by the Particle Filter. 
        rospy.Subscriber('/{}/particle_filter/viz/inferred_pose'.format(car_name), PoseStamped, purepursuit_control_node)
        
        # Subscribe to dynamic raceline from dist_finder
        rospy.Subscriber('/{}/selected_path'.format(car_name), Path, dynamic_path_callback)
        
        rospy.spin()

    except rospy.ROSInterruptException:

        pass
