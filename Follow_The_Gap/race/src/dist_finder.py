#!/usr/bin/env python

import math
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

# Configuration
DISP_THRESHOLD = 0.15  # disparity jump (m)
CAR_WIDTH = 0.3        # metres
SAFETY_MARGIN = 0.05   # add-on clearance (m)

processed_scan_pub = rospy.Publisher('/car_8/processed_scan', LaserScan, queue_size=1)
gap_info_pub = rospy.Publisher('/car_8/gap_info', Float32MultiArray, queue_size=1)


def sanitize_ranges(data):
    """Replace invalid readings using neighbouring context and clamp to [min,max]."""
    min_range = max(data.range_min, 0.05)
    max_range = data.range_max if math.isfinite(data.range_max) else min_range + 10.0

    ranges = list(data.ranges)
    n = len(ranges)

    def valid(val):
        return math.isfinite(val) and min_range <= val <= max_range

    # mark invalid values
    for i in range(n):
        r = ranges[i]
        if valid(r):
            ranges[i] = min(max(r, min_range), max_range)
        else:
            ranges[i] = None

    # forward fill using previous valid value
    prev_val = None
    for i in range(n):
        if ranges[i] is None:
            if prev_val is not None:
                ranges[i] = prev_val
        else:
            prev_val = ranges[i]

    # backward fill using next valid value
    next_val = None
    for i in reversed(range(n)):
        if ranges[i] is None:
            if next_val is not None:
                ranges[i] = next_val
        else:
            next_val = ranges[i]

    # any remaining None (all readings invalid) → max_range
    for i in range(n):
        if ranges[i] is None:
            ranges[i] = max_range

    return ranges, min_range, max_range


def calc_samples_to_extend(angle_increment, obstacle_distance):
    half_width = (CAR_WIDTH / 2.0) + SAFETY_MARGIN
    safe_dist = max(obstacle_distance, 0.05)
    span = math.atan2(half_width, safe_dist)
    return max(1, int(math.ceil(span / max(abs(angle_increment), 1e-6))))


def extend_disparities(ranges, angle_increment, disparities, min_range):
    n = len(ranges)
    for idx in disparities:
        if idx < 0 or idx + 1 >= n:
            continue

        r_close = max(min(ranges[idx], ranges[idx + 1]), min_range)
        samples = min(calc_samples_to_extend(angle_increment, r_close), n)

        if ranges[idx] < ranges[idx + 1]:
            # obstacle on right, extend leftwards
            start = idx + 1
            end = min(n, start + samples)
            for j in range(start, end):
                ranges[j] = min(ranges[j], r_close)
        else:
            # obstacle on left, extend rightwards
            end = idx + 1
            start = max(0, end - samples)
            for j in range(start, end):
                ranges[j] = min(ranges[j], r_close)


# (These constants are defined in the global scope of your script)
# CAR_WIDTH = 0.3
# SAFETY_MARGIN = 0.05

def find_best_gap(ranges, start_idx, end_idx):
    """
    Finds the best gap by prioritizing the *widest* safe gap, then
    finding the deepest point within that gap.
    """
    n = len(ranges)
    start_idx = max(0, min(n, start_idx))
    end_idx = max(start_idx, min(n, end_idx)) # Ensure end_idx >= start_idx

    if end_idx <= start_idx:
        return start_idx

    safety_radius = (CAR_WIDTH / 2.0) + SAFETY_MARGIN

    # --- Variables for finding the widest gap ---
    best_gap_start = -1
    best_gap_len = 0
    curr_gap_start = -1

    # --- Variables for the original fallback logic ---
    # Tracks the deepest point in the entire 140-degree cone
    global_max_dist = -float('inf')
    global_max_dist_idx = start_idx

    # Helper function to process a completed gap
    def consider_gap(s_idx, e_idx, best_s, best_len):
        current_len = e_idx - s_idx
        if current_len > best_len:
            # This is the new widest gap
            return s_idx, current_len
        # Keep the existing best gap
        return best_s, best_len

    # --- 1. Find the Widest Safe Gap ---
    for idx in range(start_idx, end_idx):
        dist = ranges[idx]

        # Update fallback tracker
        if dist > global_max_dist:
            global_max_dist = dist
            global_max_dist_idx = idx

        # Check if the point is safe
        if dist >= safety_radius:
            if curr_gap_start == -1:
                # Start of a new potential gap
                curr_gap_start = idx
        else:
            # Point is unsafe, so any gap we were in has now ended
            if curr_gap_start != -1:
                best_gap_start, best_gap_len = consider_gap(
                    curr_gap_start, idx, best_gap_start, best_gap_len
                )
                curr_gap_start = -1 # Reset for the next gap

    # After the loop, check if we ended inside a safe gap
    if curr_gap_start != -1:
        best_gap_start, best_gap_len = consider_gap(
            curr_gap_start, end_idx, best_gap_start, best_gap_len
        )

    # --- 2. Find the Deepest Point *within* the Widest Gap ---
    if best_gap_start != -1:
        # We found at least one safe gap
        widest_gap_end = best_gap_start + best_gap_len
        
        target_idx = best_gap_start
        max_dist_in_gap = -float('inf')

        for idx in range(best_gap_start, widest_gap_end):
            if ranges[idx] > max_dist_in_gap:
                max_dist_in_gap = ranges[idx]
                target_idx = idx
        
        return target_idx
    else:
        # --- 3. Fallback: No safe gaps were found ---
        # Revert to the original code's fallback behavior:
        # target the deepest point found anywhere in the arc.
        return global_max_dist_idx


def callback(data):
    ranges, min_range, _ = sanitize_ranges(data)
    n = len(ranges)

    forward_start = int((math.radians(-70) - data.angle_min) / data.angle_increment)
    forward_end = int((math.radians(70) - data.angle_min) / data.angle_increment)
    if forward_start > forward_end:
        forward_start, forward_end = forward_end, forward_start
    forward_start = max(0, forward_start)
    forward_end = min(n, forward_end)
    if forward_end <= forward_start:
        forward_start = 0
        forward_end = n

    disparities = []
    for i in range(max(0, forward_start - 1), min(n - 1, forward_end)):
        if abs(ranges[i + 1] - ranges[i]) > DISP_THRESHOLD:
            disparities.append(i)

    extend_disparities(ranges, data.angle_increment, disparities, min_range)

    target_idx = find_best_gap(ranges, forward_start, forward_end)
    best_angle = data.angle_min + target_idx * data.angle_increment
    best_distance = ranges[target_idx]

    processed_scan = LaserScan()
    processed_scan.header = data.header
    processed_scan.angle_min = data.angle_min
    processed_scan.angle_max = data.angle_max
    processed_scan.angle_increment = data.angle_increment
    processed_scan.time_increment = data.time_increment
    processed_scan.scan_time = data.scan_time
    processed_scan.range_min = data.range_min
    processed_scan.range_max = data.range_max
    processed_scan.intensities = data.intensities
    processed_scan.ranges = ranges
    processed_scan_pub.publish(processed_scan)

    gap_info = Float32MultiArray()
    gap_info.data = [best_angle, best_distance]
    gap_info_pub.publish(gap_info)


if __name__ == '__main__':
    rospy.init_node('dist_finder', anonymous=True)
    rospy.Subscriber("/car_8/scan", LaserScan, callback)
    rospy.spin()
