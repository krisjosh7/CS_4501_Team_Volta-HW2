#!/usr/bin/env python
import rospy
import math
import numpy as np
import sys
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
from ackermann_msgs.msg import AckermannDrive

# --- TUNING PARAMETERS ---
# Wall following / Overtaking parameters
LOOKAHEAD_DIST = 2.0        # Further lookahead = Smoother steering
WHEELBASE_LEN = 0.325
LANE_OFFSET = 0.7           # Distance to swerve (Meters)

# Speed Settings (Unitless/Percentage)
SPEED_FAST = 60.0
SPEED_SWERVE = 35.0

# Collision Box (Local Car Frame)
# How far ahead to check for obstacles?
CHECK_DIST = 2.5            # Look 2.5m ahead
CHECK_WIDTH = 0.6           # Lane width check

# HYSTERESIS (The Wobble Fix)
# How long we MUST stay in a lane after switching (Seconds)
LANE_LOCK_TIME = 1.5        

class DistFinder:
    def __init__(self):
        rospy.init_node("dist_finder")
        
        # Determine Car Name
        try:
            self.car_name = str(sys.argv[1])
        except IndexError:
            self.car_name = "car_8"

        # Subscribers
        self.scan_sub = rospy.Subscriber(f'/{self.car_name}/scan', LaserScan, self.scan_callback)
        
        # Publishers
        self.safety_pub = rospy.Publisher(f'/{self.car_name}/safety_override', Bool, queue_size=1)
        self.drive_pub  = rospy.Publisher(f'/{self.car_name}/offboard/command', AckermannDrive, queue_size=1)
        self.marker_pub = rospy.Publisher(f'/{self.car_name}/dist_finder/debug', MarkerArray, queue_size=1)

        # State
        self.current_lane = 0.0 # 0.0 = Center, + = Left, - = Right
        self.last_switch_time = rospy.Time.now()
        self.is_overtaking = False

        rospy.loginfo(f"DistFinder (Stabilized) initialized for {self.car_name}")

    def scan_callback(self, data):
        # 1. Convert LiDAR to Cartesian (Local Frame)
        # We process ALL points to find obstacles in our "Lanes"
        ranges = np.array(data.ranges)
        angles = np.linspace(data.angle_min, data.angle_max, len(ranges))
        
        # Remove garbage values
        valid = (ranges > data.range_min) & (ranges < 10.0) # optimize
        r_clean = ranges[valid]
        a_clean = angles[valid]
        
        # x = forward, y = left
        xs = r_clean * np.cos(a_clean)
        ys = r_clean * np.sin(a_clean)
        
        # 2. Check Lanes (Is there a point in the box?)
        # Box definitions: X from 0 to CHECK_DIST
        
        # Center Lane Box: Y between -0.3 and 0.3
        center_blocked = np.any((xs > 0.1) & (xs < CHECK_DIST) & (ys > -0.3) & (ys < 0.3))
        
        # Left Lane Box: Y between 0.3 and 0.9 (approx)
        left_blocked = np.any((xs > 0.1) & (xs < CHECK_DIST) & (ys > 0.3) & (ys < 0.9))
        
        # Right Lane Box: Y between -0.9 and -0.3
        right_blocked = np.any((xs > 0.1) & (xs < CHECK_DIST) & (ys > -0.9) & (ys < -0.3))

        # 3. Decision Logic (With Hysteresis)
        now = rospy.Time.now()
        time_since_switch = (now - self.last_switch_time).to_sec()
        
        # Default: stick to current decision
        target_lane = self.current_lane
        
        if time_since_switch > LANE_LOCK_TIME:
            # We are allowed to switch lanes now
            
            if self.current_lane == 0.0:
                # We are in Center. Is it blocked?
                if center_blocked:
                    # Swerve! Prefer Left, then Right.
                    if not left_blocked:
                        target_lane = LANE_OFFSET # Go Left
                    elif not right_blocked:
                        target_lane = -LANE_OFFSET # Go Right
                    else:
                        # Panic: All blocked. Stick to center and brake (handled in speed)
                        target_lane = 0.0 
            else:
                # We are currently Swerving (Left or Right). 
                # Can we return to center?
                if not center_blocked:
                    target_lane = 0.0 # Return to optimal
                else:
                    # Center still blocked. Is our current swerve lane blocked?
                    # If yes, try the OTHER swerve lane.
                    if (self.current_lane > 0 and left_blocked) and not right_blocked:
                        target_lane = -LANE_OFFSET
                    elif (self.current_lane < 0 and right_blocked) and not left_blocked:
                        target_lane = LANE_OFFSET

        # 4. Apply Switch
        if target_lane != self.current_lane:
            self.current_lane = target_lane
            self.last_switch_time = now
            # Log it so you know why it switched
            rospy.loginfo(f"SWITCHING LANE to {self.current_lane}")

        # 5. Handover Control
        if self.current_lane == 0.0:
            # Center is clear/active -> Let PurePursuit drive
            self.safety_pub.publish(False)
            self.is_overtaking = False
        else:
            # We are swerving -> TAKE CONTROL
            self.safety_pub.publish(True)
            self.is_overtaking = True
            
            # Simple Reactive Controller (Follow the offset)
            # Instead of complex Pure Pursuit on a fake path, we just steer towards the lane offset
            # This is "Follow the Gap" style simplified math for stability
            
            # Geometry: We want to be at Y = current_lane
            # We are at Y = 0 (Local frame)
            # Error = current_lane.
            # Steer = P * Error
            
            # Pure Pursuit Logic in Local Frame:
            # Target Point: x = LOOKAHEAD_DIST, y = current_lane
            alpha = math.atan2(self.current_lane, LOOKAHEAD_DIST)
            steer_rad = math.atan2(2.0 * WHEELBASE_LEN * math.sin(alpha), LOOKAHEAD_DIST)
            
            # Convert to servo range
            steer_cmd = (steer_rad / 0.4189) * 100.0
            steer_cmd = max(-100, min(100, steer_cmd))
            
            msg = AckermannDrive()
            msg.steering_angle = steer_cmd
            msg.speed = SPEED_SWERVE # Slow down slightly to handle the swerve
            
            self.drive_pub.publish(msg)

        # 6. Debug Visualization
        self.publish_debug_markers(center_blocked, left_blocked, right_blocked)

    def publish_debug_markers(self, c, l, r):
        # Visualize the 3 boxes. Red = Blocked, Green = Clear
        m_array = MarkerArray()
        
        lanes = [
            (0.0, c), # Center
            (0.6, l), # Left
            (-0.6, r) # Right
        ]
        
        for i, (y_off, blocked) in enumerate(lanes):
            marker = Marker()
            marker.header.frame_id = "laser" # Local frame!
            marker.header.stamp = rospy.Time.now()
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            marker.pose.position.x = CHECK_DIST / 2.0
            marker.pose.position.y = y_off
            marker.pose.position.z = 0.0
            
            marker.scale.x = CHECK_DIST
            marker.scale.y = 0.4
            marker.scale.z = 0.1
            
            marker.color.a = 0.5
            if blocked:
                marker.color.r = 1.0 # Red
            else:
                marker.color.g = 1.0 # Green
                
            if self.is_overtaking and abs(self.current_lane - y_off) < 0.1:
                marker.color.b = 1.0 # Blue if we are currently choosing this lane
                marker.color.a = 0.8
                
            m_array.markers.append(marker)
            
        self.marker_pub.publish(m_array)

if __name__ == "__main__":
    try:
        DistFinder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass