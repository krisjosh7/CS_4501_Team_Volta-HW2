#!/usr/bin/env python

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from tf.transformations import quaternion_from_euler

# Constants from dist_finder.py
CAR_WIDTH = 0.33
CAR_LENGTH = 0.50
DISP_THRESHOLD = 0.1

# Publishers for visualization
target_pub = rospy.Publisher("/target_point", Marker, queue_size=2)
footprint_pub = rospy.Publisher("/car_footprint", Marker, queue_size=2)
disparities_pub = rospy.Publisher("/disparity_markers", MarkerArray, queue_size=2)
steering_pub = rospy.Publisher("/steering_arrow", Marker, queue_size=2)


def callback(data):
    # 1. Clean ranges data
    ranges = list(data.ranges)
    n = len(ranges)
    for i in range(n):
        if math.isnan(ranges[i]) or math.isinf(ranges[i]):
            ranges[i] = data.range_max
    
    # 2. Find disparities
    disparities = []
    for i in range(n - 1):
        if abs(ranges[i + 1] - ranges[i]) > DISP_THRESHOLD:
            disparities.append(i)
    
    # 3. Focus on forward-facing range (-90° to +90°)
    start_angle = int(((-90 * math.pi / 180) - data.angle_min) / data.angle_increment)
    end_angle = int(((90 * math.pi / 180) - data.angle_min) / data.angle_increment)
    front_ranges = ranges[start_angle:end_angle]
    
    # 4. Find the target point (deepest gap)
    max_idx = front_ranges.index(max(front_ranges))
    best_angle = data.angle_min + (start_angle + max_idx) * data.angle_increment
    max_range = front_ranges[max_idx]
    
    # Visualization 1: Car Footprint
    footprint = Marker()
    footprint.header.frame_id = "car_9_laser"
    footprint.header.stamp = rospy.Time.now()
    footprint.type = Marker.LINE_STRIP
    footprint.id = 0
    footprint.scale.x = 0.05  # line width
    footprint.color.r = 1.0
    footprint.color.g = 1.0
    footprint.color.b = 1.0
    footprint.color.a = 1.0
    
    # Create rectangle corners
    pts = [
        (-CAR_LENGTH/2, -CAR_WIDTH/2, 0),
        (CAR_LENGTH/2, -CAR_WIDTH/2, 0),
        (CAR_LENGTH/2, CAR_WIDTH/2, 0),
        (-CAR_LENGTH/2, CAR_WIDTH/2, 0),
        (-CAR_LENGTH/2, -CAR_WIDTH/2, 0)
    ]
    for pt in pts:
        p = Point()
        p.x, p.y, p.z = pt
        footprint.points.append(p)
    
    footprint_pub.publish(footprint)
    
    # Visualization 2: Disparity Points
    markers = MarkerArray()
    for idx, disp_idx in enumerate(disparities):
        angle = data.angle_min + disp_idx * data.angle_increment
        r = ranges[disp_idx]
        
        marker = Marker()
        marker.header.frame_id = "car_9_laser"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.SPHERE
        marker.id = idx
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        marker.pose.position.x = r * math.cos(angle)
        marker.pose.position.y = r * math.sin(angle)
        marker.pose.position.z = 0
        
        markers.markers.append(marker)
    
    disparities_pub.publish(markers)
    
    # Visualization 3: Target Point (deepest gap)
    target = Marker()
    target.header.frame_id = "car_9_laser"
    target.header.stamp = rospy.Time.now()
    target.type = Marker.SPHERE
    target.id = 0
    target.scale.x = target.scale.y = target.scale.z = 0.3
    target.color.r = 0.0
    target.color.g = 1.0
    target.color.b = 0.0
    target.color.a = 1.0
    
    target.pose.position.x = max_range * math.cos(best_angle)
    target.pose.position.y = max_range * math.sin(best_angle)
    target.pose.position.z = 0
    
    target_pub.publish(target)
    
    # Visualization 4: Steering Direction
    steering = Marker()
    steering.header.frame_id = "car_9_laser"
    steering.header.stamp = rospy.Time.now()
    steering.type = Marker.ARROW
    steering.id = 0
    steering.scale.x = 1.0  # arrow length
    steering.scale.y = 0.1  # arrow width
    steering.scale.z = 0.1  # arrow height
    steering.color.r = 1.0
    steering.color.g = 1.0
    steering.color.b = 0.0
    steering.color.a = 1.0
    
    # Arrow points from origin to target direction
    steering.points = [
        Point(0, 0, 0),
        Point(
            math.cos(best_angle),
            math.sin(best_angle),
            0
        )
    ]
    
    steering_pub.publish(steering)

   
if __name__=='__main__':
    rospy.init_node("rviz_test", anonymous=False)
    rospy.loginfo("Starting visualization node for gap following algorithm...")
    sub = rospy.Subscriber("/car_9/scan", LaserScan, callback)
    rospy.spin()

