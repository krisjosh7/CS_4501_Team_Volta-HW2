#!/usr/bin/env python
import math
import rospy
from std_msgs.msg import Float32MultiArray
from ackermann_msgs.msg import AckermannDrive

# Control parameters
servo_offset = 0.0  # zero correction offset in case servo is misaligned


# This code can input desired velocity from the user.
# velocity must be between [0,100] to move forward.
# The following velocity values correspond to different speed profiles.
# 15: Very Slow (Good for debug mode)
# 25: Slow and steady
# 35: Nice Autonomous Pace
# > 40: Careful, what you do here. Only use this if your autonomous steering is very reliable.
vel_input = 0.0	#TODO

# Publishers and subscribers
command_pub = rospy.Publisher('/car_8/offboard/command', AckermannDrive, queue_size=1)

def gap_callback(msg):
    # Extract gap information
    best_angle = msg.data[0]  # in radians
    gap_distance = msg.data[1]  # distance to the gap TODO: implement dynamic velocity scaling
    
    # Convert steering angle --> control signal range [-100, 100]
    target_angle = (math.degrees(best_angle))

    if ((target_angle) >= 30):
        steering = 100
    elif(target_angle <= -30):
        steering = -100
    else:
        steering = target_angle * (100/25)
    
    # Add servo offset if needed
    steering += servo_offset
    
    # Determine velocity based on gap distance
    velocity = 30 - (abs(steering) / 100) * 18

    if(abs(gap_distance) >= 3.5 and abs(steering) <=  20):
        velocity = 35
    elif(abs(steering) - 40 <= 20):
        velocity = 25
    elif(abs(steering) - 80 <= 20):
        velocity = 20

    # Create and publish command
    command = AckermannDrive()
    command.steering_angle = steering
    command.speed = velocity
    command_pub.publish(command)

if __name__ == '__main__':
    try:
        rospy.init_node('gap_follower')
        rospy.loginfo("Starting gap follower node...")
        
        # Subscribe to the gap information
        rospy.Subscriber("/car_8/gap_info", Float32MultiArray, gap_callback)
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
