#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive

# Some useful variable declarations.

servo_offset = 0.0	# zero correction offset in case servo is misaligned and has a bias in turning.

angle_range = 240	# Hokuyo 4LX has 240 degrees FoV for scan
forward_projection = 1.5	# distance (in m) that we project the car forward for correcting the error. You have to adjust this.
desired_distance = 0.9	# distance from the wall (in m). (defaults to right wall). You need to change this for the track

disp_threshold = 0.2

vel = 0.0	# this vel variable IS really used here.
error = 0.0		# initialize the error
car_length = 0.50 # Traxxas Rally is 20 inches or 0.5 meters. Useful variable.

car_width = 0.23

# Handle to the publisher that will publish on the error topic, messages of the type 'pid_input'
command_pub = rospy.Publisher('/car_8/offboard/command', AckermannDrive, queue_size = 1)


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

    #Get lidar ranges and clean them
    ranges = list(data.ranges)
    n = len(ranges)
    for i in range(n):
        if math.isnan(ranges[i]) or math.isinf(ranges[i]):
            ranges[i] = data.range_max

    #Find disparities
    disparities = []
    for i in range(n - 1):
        if abs(ranges[i + 1] - ranges[i]) > disp_threshold:
            disparities.append(i)

    #Extend obstacles near disparities
    for i in disparities:
        r_close = min(ranges[i], ranges[i + 1])
        # r_far = max(ranges[i], ranges[i + 1])

        # how many samples correspond to half the car width at this distance
        theta = abs(data.angle_increment)
        half_width = (car_width / 2.0) + 0.05  # tolerance
        ang_span = math.atan2(half_width, max(r_close, 1e-3))
        samples_to_extend = int(ang_span / max(theta, 1e-6))
        if samples_to_extend <= 0:
            continue

        # overwrite from the far side toward same direction
        if ranges[i] > ranges[i + 1]:
            # obstacle closer on right extend to the right
            for j in range(i + 1, min(i + 1 + samples_to_extend, n)):
                ranges[j] = min(ranges[j], r_close)
        else:
            # obstacle closer on left extend to the left
            for j in range(max(0, i - samples_to_extend), i+1):
                ranges[j] = min(ranges[j], r_close)

    #Focus only on forward-facing range (-90 to +90)
    start_angle = int((math.radians(-90) - data.angle_min) / data.angle_increment)
    end_angle   = int((math.radians(90) - data.angle_min) / data.angle_increment)
    front_ranges = ranges[start_angle:end_angle]

    #Find the index of the farthest point (deepest gap)
    max_idx = front_ranges.index(max(front_ranges))
    best_angle = (data.angle_min + (start_angle + max_idx)) * data.angle_increment

    #Convert steering angle control signal range [-100, 100]
    steering = math.degrees(best_angle)
    steering = max(-100, min(100, (steering / 90.0) * 100))

    # (optional) Adjust velocity based on obstacle proximity
    # min_front = min(front_ranges)
    # if min_front < 0.2:
    #     vel = 10  # stop if too close
    # else:
    #     vel = 25
    vel = 15

    # Publish the control message
    command = AckermannDrive()
    command.steering_angle = steering
    command.speed = vel
    command_pub.publish(command)



if __name__ == '__main__':

    print("Hokuyo LIDAR node started")
    rospy.init_node('dist_finder',anonymous = True)
	# TODO: Make sure you are subscribing to the correct car_x/scan topic on your racecar
    rospy.Subscriber("/car_8/scan",LaserScan,callback)
    rospy.spin()
