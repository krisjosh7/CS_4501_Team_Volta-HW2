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
    
    # Convert steering angle → control signal range [-100, 100]
    steering = math.degrees(best_angle)
    steering = max(-100, min(100, (steering / 90.0) * 100))
    
    # Add servo offset if needed
    steering += servo_offset
    
    # Determine velocity based on gap distance
    velocity = 15
        
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
