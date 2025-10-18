#!/usr/bin/env python
import math
import rospy
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDrive

# PID Control Params
kp = 0.0 #TODO
kd = 0.0 #TODO
ki = 0.0 #TODO
servo_offset = 0.0	# zero correction offset in case servo is misaligned and has a bias in turning.
prev_error = 0.0
integral_error = 0.0
prev_time = None


# This code can input desired velocity from the user.
# velocity must be between [0,100] to move forward.
# The following velocity values correspond to different speed profiles.
# 15: Very Slow (Good for debug mode)
# 25: Slow and steady
# 35: Nice Autonomous Pace
# > 40: Careful, what you do here. Only use this if your autonomous steering is very reliable.

vel_input = 0.0	#TODO

CONTROL_HZ = 50.0        # run controller at 50 Hz
TS = 1.0 / CONTROL_HZ    # sample time (seconds)

# Steering/clamping configuration
STEERING_LIMIT = 30.0   # degrees (change to your vehicle's expected unit or keep larger if your vehicle expects different scale)
INTEGRAL_LIMIT = 100.0  # anti-windup clamp for integral term

# Publisher for moving the car.
# TODO: Use the coorect topic /car_x/offboard/command. The multiplexer listens to this topic
command_pub = rospy.Publisher('/car_8/offboard/command', AckermannDrive, queue_size = 1)

def control(data):
	global prev_error
	global vel_input
	global kp
	global kd
	global angle

	print("PID Control Node is Listening to error")

	# Timing
	global integral_error, prev_time
	now = rospy.Time.now()
	if prev_time is None:
		# first callback, use nominal sample time
		dt = TS
	else:
		dt = (now - prev_time).to_sec()
		if dt <= 0.0:
			dt = TS
	prev_time = now

	#TODO: Use kp, ki & kd to implement a PID controller

	kp_f = float(kp)
	kd_f = float(kd)
	ki_f = float(ki)
	v_cmd = float(vel_input)

	# 1. Read the error
	try:
		err = float(data.pid_error)
	except Exception:
		rospy.logwarn("Invalid pid_error received; ignoring")
		return

	# Integrate & differentiate with anti-windup
	integral_error += err * dt
	# clamp integral to avoid windup
	if integral_error > INTEGRAL_LIMIT:
		integral_error = INTEGRAL_LIMIT
	elif integral_error < -INTEGRAL_LIMIT:
		integral_error = -INTEGRAL_LIMIT

	derivative = 0.0
	# prev_error may not be defined on first run; use it safely
	try:
		derivative = (err - prev_error) / dt
	except Exception:
		derivative = 0.0

	prev_error = err

	# PID output
	pid_out = kp_f * err + ki_f * integral_error + kd_f * derivative

	# compute steering angle command (apply servo offset)
	angle = servo_offset + pid_out

	# Clamp steering angle to safe bounds
	if angle > STEERING_LIMIT:
		angle = STEERING_LIMIT
	elif angle < -STEERING_LIMIT:
		angle = -STEERING_LIMIT

	# Prepare Ackermann command
	command = AckermannDrive()

	# TODO: Make sure the steering value is within bounds [-100,100]
	if angle > 100.0:
		angle = 100.0
	elif angle < -100.0:
		angle = -100.0
	command.steering_angle = angle

	# Clamp velocity
	if v_cmd < 0.0:
		v_cmd = 0.0
	elif v_cmd > 100.0:
		v_cmd = 100.0
	command.speed = v_cmd

	# Publish
	command_pub.publish(command)

if __name__ == '__main__':

    # This code tempalte asks for the values for the gains from the user upon start, but you are free to set them as ROS parameters as well.
	global kp
	global kd
	global ki
	global vel_input
	kp = input("Enter Kp Value: ")
	kd = input("Enter Kd Value: ")
	ki = input("Enter Ki Value: ")
	vel_input = input("Enter desired velocity: ")
	rospy.init_node('pid_controller', anonymous=True)
    # subscribe to the error topic
	rospy.Subscriber("error", pid_input, control)
	rospy.spin()
