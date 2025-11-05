#!/usr/bin/env python

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

# Constants for LIDAR processing
angle_range = 240  # Hokuyo 4LX has 240 degrees FoV for scan
disp_threshold = 0.15  # Threshold for detecting disparities
car_width = 0.2  # Car width in meters

# Publishers for processed data
processed_scan_pub = rospy.Publisher('/car_8/processed_scan', LaserScan, queue_size=1)
gap_info_pub = rospy.Publisher('/car_8/gap_info', Float32MultiArray, queue_size=1)

def calcSamplesToExtend(theta, radius):
        half_width = (car_width / 2.0) + 0.05  # tolerance
        theta_span_half = half_width/radius
        return int(math.ceil(theta_span_half / theta))

def getAngleFromIndex(data, idx):
    angle_rad = data.angle_min + idx * data.angle_increment
    return math.degrees(angle_rad) + 90

def getRange(data,angle):
	# data: single message from topic /scan
    # angle: between -30 to 210 degrees, where 0 degrees is directly to the right, and 90 degrees is directly in front
    # Outputs length in meters to object with angle in lidar scan field of view
    # Make sure to take care of NaNs etc.
	
	angle_rad = math.radians((angle-90))
	idx = int(round((angle_rad - data.angle_min) / data.angle_increment))

	n = len(data.ranges)
	if idx < 0:
		idx = 0
	elif idx >= n:
		idx = n - 1

	def valid(r):
		return (
            r is not None
            and not math.isnan(r)
            and not math.isinf(r)
            and data.range_min <= r <= data.range_max
        )

	r = data.ranges[idx]
	if valid(r):
		return r
	
	return data.range_max

def callback(data):
    global vel

    ranges = list(data.ranges)

    start_angle = int((math.radians(-90) - data.angle_min) / data.angle_increment)
    end_angle   = int((math.radians(90) - data.angle_min) / data.angle_increment)
   

    #Get lidar ranges and clean them
    
    n = len(ranges)

    for i in range(start_angle, end_angle):
        if math.isnan(ranges[i]) or math.isinf(ranges[i]):
            ranges[i] = 5.0

    print(ranges)
			
    #Find disparities
    disparities = []
    for i in range(start_angle, end_angle):
        if abs(ranges[i + 1] - ranges[i]) > disp_threshold:
            disparities.append(i)
    
    disp_angles = []
    for i in range(len(disparities)):
          disp_angles.append(int(getAngleFromIndex(data, disparities[i])))

    #print(disp_angles)

    for i in disparities:
        r_close = min(ranges[i], ranges[i + 1])

        if r_close <= data.range_min:
             continue

        # how many samples correspond to half the car width at this distance
        samples_to_extend = min(calcSamplesToExtend(data.angle_increment, r_close), n)
        if samples_to_extend < 0:
            continue

        # overwrite from the far side toward same direction
        if ranges[i] < ranges[i + 1]:
            # obstacle closer on right extend into the gap on the left
            for j in range(i + 1, min(i + 1 + samples_to_extend, n)):
                ranges[j] = min(ranges[j], r_close)
        else:
            # obstacle closer on left extend into the gap on the right
            for j in range(max(0, i - samples_to_extend), i+1):
                ranges[j] = min(ranges[j], r_close)

    # Find the index of the farthest point (deepest gap)
    # max_idx = ranges.index(max(ranges))
    # best_angle = getAngleFromIndex(data, max_idx)
                
    print(ranges)

    best_gap_idx = None
    best_gap_dist = -float('inf')
    best_gap_len = 0
    curr_start_idx = None
    curr_len = 0
    maxDist = -float('inf')
    max_dist_idx = None
    half_width = (car_width / 2.0) + 0.05 

    def update_best_gap(best_idx, best_dist, best_len, start_idx, length):
        if start_idx is None or length <= 0:
            return best_idx, best_dist, best_len
        mid_idx = start_idx + length // 2
        if not (0 <= mid_idx < n):
            return best_idx, best_dist, best_len
        dist_mid = ranges[mid_idx]
        if (
            dist_mid > best_dist
            or (abs(dist_mid - best_dist)  < 0.0001 and length > best_len)
        ):
            return mid_idx, dist_mid, length
        return best_idx, best_dist, best_len

    for beam_idx in range(start_angle, end_angle):
        if 0<= beam_idx < n:
            dist = ranges[beam_idx]
            if dist > maxDist:
                maxDist = dist
                max_dist_idx = beam_idx

            if dist > half_width:
                if curr_start_idx is None:
                    curr_start_idx = beam_idx
                    curr_len = 1
                else:
                    curr_len += 1
            
            else:
                best_gap_idx, best_gap_dist, best_gap_len = update_best_gap(
                    best_gap_idx, best_gap_dist, best_gap_len, curr_start_idx, curr_len
                )
                curr_start_idx = None
                curr_len = 0
        else:
            best_gap_idx, best_gap_dist, best_gap_len = update_best_gap(
                best_gap_idx, best_gap_dist, best_gap_len, curr_start_idx, curr_len
            )
            curr_start_idx = None
            curr_len = 0
    best_gap_idx, best_gap_dist, best_gap_len = update_best_gap(
        best_gap_idx, best_gap_dist, best_gap_len, curr_start_idx, curr_len
    )
    
    if best_gap_idx is not None:
        target_idx = best_gap_idx
    elif max_dist_idx is not None:
        target_idx = max_dist_idx
    else:   
        target_idx = max(0, min(n-1, start_angle))
    
    target_idx = max(0, min(n-1, target_idx))
    best_angle = data.angle_min + target_idx * data.angle_increment
    best_distance = ranges[target_idx]


    # Publish processed scan
    processed_scan = LaserScan()
    processed_scan.header = data.header
    processed_scan.angle_min = data.angle_min
    processed_scan.intensities = data.intensities
    processed_scan.angle_max = data.angle_max
    processed_scan.angle_increment = data.angle_increment
    processed_scan.time_increment = data.time_increment
    processed_scan.scan_time = data.scan_time
    processed_scan.range_min = data.range_min
    processed_scan.range_max = data.range_max

    processed_scan.ranges = ranges
    processed_scan_pub.publish(processed_scan)

    # Publish gap info
    gap_info = Float32MultiArray()
    gap_info.data = [best_angle, best_distance]  # angle and distance to gap
    gap_info_pub.publish(gap_info)

if __name__ == '__main__':

    print("Hokuyo LIDAR node started")
    rospy.init_node('dist_finder',anonymous = True)
	# TODO: Make sure you are subscribing to the correct car_x/scan topic on your racecar
    rospy.Subscriber("/car_8/scan", LaserScan, callback)
    rospy.spin()
