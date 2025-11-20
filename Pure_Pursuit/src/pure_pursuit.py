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

def construct_path():
    # Function to construct the path from a CSV file
    # Use the script's directory to find the csv file
    file_path = os.path.expanduser('~/depend_ws/src/f1tenth_purepursuit/path/{}.csv'.format(trajectory_name))
    with open(file_path) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter = ',')
        for waypoint in csv_reader:
            plan.append(waypoint)

    # Convert string coordinates to floats and calculate path resolution
    for index in range(0, len(plan)):
        for point in range(0, len(plan[index])):
            plan[index][point] = float(plan[index][point])

    for index in range(1, len(plan)):
         dx = plan[index][0] - plan[index-1][0]
         dy = plan[index][1] - plan[index-1][1]
         path_resolution.append(math.sqrt(dx*dx + dy*dy))

    # Create and publish the path for visualization
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

    # Obtain the current position of the race car from the inferred_pose message
    odom_x = data.pose.position.x
    odom_y = data.pose.position.y


    # TODO 1: The reference path is stored in the 'plan' array.
    # Your task is to find the base projection of the car on this reference path.
    # The base projection is defined as the closest point on the reference path to the car's current position.
    # Calculate the index and position of this base projection on the reference path.
    
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
    

    # TODO 2: You need to tune the value of the lookahead_distance
    # This is a key tuning parameter for pure pursuit.
    # A larger lookahead distance will make the car's path smoother but might cause it to cut corners.
    # A smaller lookahead distance will make the car follow the path more closely but can lead to oscillations.
    lookahead_distance = 1.2 # Tunable parameter


    # TODO 3: Utilizing the base projection found in TODO 1, your next task is to identify the goal or target point for the car.
    # This target point should be determined based on the path and the base projection you have already calculated.
    # The target point is a specific point on the reference path that the car should aim towards - lookahead distance ahead of the base projection on the reference path.
    # Calculate the position of this goal/target point along the path.

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


    # TODO 4: Implement the pure pursuit algorithm to compute the steering angle given the pose of the car, target point, and lookahead distance.
    
    # Transform the target point to the vehicle's coordinate frame
    dx = target_x - odom_x
    dy = target_y - odom_y
    
    # Rotate the vector to the target point by -heading to get it in the car's frame
    x_rel = dx * math.cos(heading) + dy * math.sin(heading)
    y_rel = -dx * math.sin(heading) + dy * math.cos(heading)

    # Calculate the steering angle using the pure pursuit formula.
    # alpha is the angle between the car's heading and the lookahead vector.
    alpha = math.atan2(y_rel, x_rel)
    
    # The steering angle is given by arctan(2 * L * sin(alpha) / Ld)
    # where L is the wheelbase, and Ld is the distance to the target.
    actual_lookahead_dist = math.sqrt(dx*dx + dy*dy)
    steering_angle_rad = math.atan2(2.0 * WHEELBASE_LEN * math.sin(alpha), actual_lookahead_dist)


    # TODO 5: Ensure that the calculated steering angle is within the STEERING_RANGE and assign it to command.steering_angle
    
    # Assume a maximum physical steering angle (e.g., 30 degrees)
    max_steering_rad = math.radians(30) 
    
    # Clamp the steering angle to the physical limits of the car
    steering_angle_rad = max(-max_steering_rad, min(max_steering_rad, steering_angle_rad))
    
    # Scale the radian value to the specified STEERING_RANGE [-100, 100]
    command.steering_angle = (steering_angle_rad / max_steering_rad) * STEERING_RANGE
    

    # TODO 6: Implement Dynamic Velocity Scaling instead of a constant speed
    
    # Decrease speed for sharper turns.
    max_speed = 4.0  # m/s, tune this
    min_speed = 1.5  # m/s, tune this
    
    # Linearly scale speed based on the magnitude of the steering angle
    abs_steering = abs(command.steering_angle)
    command.speed = max_speed - (abs_steering / STEERING_RANGE) * (max_speed - min_speed)

    command_pub.publish(command)

    # Visualization code
    # Make sure the following variables are properly defined in your TODOs above:
    # - odom_x, odom_y: Current position of the car
    # - pose_x, pose_y: Position of the base projection on the reference path
    # - target_x, target_y: Position of the goal/target point

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
        if not plan:
            rospy.loginfo('obtaining trajectory')
            construct_path()

        # This node subsribes to the pose estimate provided by the Particle Filter. 
        # The message type of that pose message is PoseStamped which belongs to the geometry_msgs ROS package.
        rospy.Subscriber('/{}/particle_filter/viz/inferred_pose'.format(car_name), PoseStamped, purepursuit_control_node)
        rospy.spin()

    except rospy.ROSInterruptException:

        pass
