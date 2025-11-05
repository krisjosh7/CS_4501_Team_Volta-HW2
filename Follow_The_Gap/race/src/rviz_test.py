#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

# Constants
CAR_WIDTH = 0.33
CAR_LENGTH = 0.50

# Publishers for visualization
target_pub = rospy.Publisher("/target_point", Marker, queue_size=2)
footprint_pub = rospy.Publisher("/car_footprint", Marker, queue_size=2)
disparities_pub = rospy.Publisher("/disparity_markers", MarkerArray, queue_size=2)
steering_pub = rospy.Publisher("/steering_arrow", Marker, queue_size=2)

# Global variables to store latest data
latest_scan = None
latest_angle = None
latest_distance = None
prev_disparity_count = 0


def scan_callback(data):
    """Handle processed scan data"""
    global latest_scan
    latest_scan = data

def gap_callback(data):
    """Handle gap information"""
    global latest_angle, latest_distance
    latest_angle = data.data[0]  # in radians
    latest_distance = data.data[1]  # distance to gap

def timer_callback(event):
    """Update visualizations at fixed rate"""
    if latest_scan is None or latest_angle is None:
        return
    
    # Visualization 1: Car Footprint
    footprint = Marker()
    footprint.header.frame_id = "car_8_laser"  # Changed to car_8
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
    
    # Visualization 2: Disparity Points from processed scan
    global prev_disparity_count
    if latest_scan:
        markers = MarkerArray()
        ranges = latest_scan.ranges
        disparity_indices = []
        for i in range(len(ranges) - 1):
            if abs(ranges[i + 1] - ranges[i]) > 0.1:  # disparity threshold
                disparity_indices.append(i)
        # Create markers for current disparities
        for idx, i in enumerate(disparity_indices):
            angle = latest_scan.angle_min + i * latest_scan.angle_increment
            r = ranges[i]
            marker = Marker()
            marker.header.frame_id = "car_8_laser"
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
        # Delete any leftover markers from previous frame
        for idx in range(len(disparity_indices), prev_disparity_count):
            marker = Marker()
            marker.header.frame_id = "car_8_laser"
            marker.header.stamp = rospy.Time.now()
            marker.action = Marker.DELETE
            marker.id = idx
            markers.markers.append(marker)
        disparities_pub.publish(markers)
        prev_disparity_count = len(disparity_indices)
    
    # Visualization 3: Target Point (deepest gap)
    if latest_distance is not None and latest_angle is not None:
        target = Marker()
        target.header.frame_id = "car_8_laser"  # Changed to car_8
        target.header.stamp = rospy.Time.now()
        target.type = Marker.SPHERE
        target.id = 0
        target.scale.x = target.scale.y = target.scale.z = 0.3
        target.color.r = 0.0
        target.color.g = 1.0
        target.color.b = 0.0
        target.color.a = 1.0
        
        target.pose.position.x = latest_distance * math.cos(latest_angle)
        target.pose.position.y = latest_distance * math.sin(latest_angle)
        target.pose.position.z = 0
        
        target_pub.publish(target)
        
        # Visualization 4: Steering Direction
        steering = Marker()
        steering.header.frame_id = "car_8_laser"  # Changed to car_8
        steering.header.stamp = rospy.Time.now()
        steering.type = Marker.ARROW
        steering.id = 0
        steering.scale.x = 0.3  # arrow length
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
                math.cos(latest_angle),
                math.sin(latest_angle),
                0
            )
        ]
        
        steering_pub.publish(steering)

   
if __name__=='__main__':
    rospy.init_node("rviz_test", anonymous=False)
    rospy.loginfo("Starting visualization node for gap following algorithm...")
    
    # Subscribe to processed data
    scan_sub = rospy.Subscriber("/car_8/processed_scan", LaserScan, scan_callback)
    gap_sub = rospy.Subscriber("/car_8/gap_info", Float32MultiArray, gap_callback)
    
    # Create timer for visualization updates (10Hz)
    timer = rospy.Timer(rospy.Duration(0.1), timer_callback)
    
    rospy.spin()

