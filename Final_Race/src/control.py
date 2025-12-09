#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float32MultiArray
from ackermann_msgs.msg import AckermannDrive

# --- CONFIGURATION ---
CAR_NAME = "car_8"
MAX_STEERING_ANGLE = 0.4189 # ~24 degrees in radians
STEERING_RANGE = 100.0 # Servo range [-100, 100]

class Control:
    def __init__(self):
        rospy.init_node('control_node')
        
        self.drive_pub = rospy.Publisher('/car_8/offboard/command', AckermannDrive, queue_size=1)
        self.gap_sub = rospy.Subscriber('/car_8/gap_info', Float32MultiArray, self.gap_callback)
        
        rospy.loginfo("Control node initialized.")

    def gap_callback(self, msg):
        # msg.data = [steering_angle_rad, target_velocity]
        steering_angle_rad = msg.data[0]
        target_v = 30 #CHANGE LATER
        
        # 1. Convert Radians to Servo Command [-100, 100]
        # Clamp to physical limits first
        steering_angle_rad = max(-MAX_STEERING_ANGLE, min(MAX_STEERING_ANGLE, steering_angle_rad))
        
        # Map to servo range
        servo_steering = (steering_angle_rad / MAX_STEERING_ANGLE) * STEERING_RANGE
        
        # 2. Dynamic Speed Control (Optional override of target_v)
        # If the turn is sharp, slow down regardless of what the raceline says
        if abs(servo_steering) > 50:
            target_v = min(target_v, 2.0) # Slow down for sharp turns
        elif abs(servo_steering) > 30:
            target_v = min(target_v, 3.0)
            
        # 3. Publish Command
        drive = AckermannDrive()
        drive.speed = target_v
        drive.steering_angle = servo_steering
        
        self.drive_pub.publish(drive)

if __name__ == '__main__':
    try:
        Control()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
