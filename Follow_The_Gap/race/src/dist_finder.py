#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

# Constants for LIDAR processing
angle_range = 240  # Hokuyo 4LX has 240 degrees FoV for scan
disp_threshold = 0.15  # Threshold for detecting disparities
car_width = 0.2  # Car width in meters
safety_margin = 0.05  # Extra clearance around the car

# Publishers for processed data
processed_scan_pub = rospy.Publisher('/car_8/processed_scan', LaserScan, queue_size=1)
gap_info_pub = rospy.Publisher('/car_8/gap_info', Float32MultiArray, queue_size=1)

def calc_samples_to_extend(angle_increment, obstacle_distance):
    """Return how many beams we should inflate across a disparity."""
    half_width = (car_width / 2.0) + safety_margin
    safe_distance = max(obstacle_distance, 0.05)
    span = math.atan2(half_width, safe_distance)
    return max(1, int(math.ceil(span / max(abs(angle_increment), 1e-6))))

def angle_to_index(data, angle_rad):
    """Convert an angle (radians) in the laser frame to an index in the scan array."""
    return int(round((angle_rad - data.angle_min) / data.angle_increment))

def sanitize_ranges(data):
    """Replace invalid readings using neighbouring context and clamp results."""
    min_range = max(data.range_min, 0.05)
    max_range = data.range_max if math.isfinite(data.range_max) else min_range + 10.0

    ranges = list(data.ranges)
    n = len(ranges)

    def is_valid(val):
        return math.isfinite(val) and min_range <= val <= max_range

    # First pass: clamp valid values, mark invalid as None
    for i in range(n):
        r = ranges[i]
        if is_valid(r):
            ranges[i] = min(max(r, min_range), max_range)
        else:
            ranges[i] = None

    # Second pass: interpolate invalid runs using nearest valid neighbours
    i = 0
    while i < n:
        if ranges[i] is not None:
            i += 1
            continue

        run_start = i
        while i < n and ranges[i] is None:
            i += 1
        run_end = i  # exclusive

        left_idx = run_start - 1
        right_idx = run_end

        left_val = ranges[left_idx] if left_idx >= 0 else None
        right_val = ranges[right_idx] if right_idx < n else None

        if left_val is not None and right_val is not None:
            span = right_idx - left_idx
            for offset, idx in enumerate(range(run_start, run_end), start=1):
                blend = left_val + (right_val - left_val) * (offset / span)
                ranges[idx] = min(max(blend, min_range), max_range)
        elif left_val is not None:
            for idx in range(run_start, run_end):
                ranges[idx] = left_val
        elif right_val is not None:
            for idx in range(run_start, run_end):
                ranges[idx] = right_val
        else:
            fill = max_range
            for idx in range(run_start, run_end):
                ranges[idx] = fill

    return ranges, min_range, max_range

def extend_disparities(ranges, angle_increment, disparities, min_range):
    """Inflate obstacles near disparity jumps so narrow gaps disappear."""
    n = len(ranges)
    for i in disparities:
        if i < 0 or i + 1 >= n:
            continue

        r_close = min(ranges[i], ranges[i + 1])
        r_close = max(r_close, min_range)

        samples = min(calc_samples_to_extend(angle_increment, r_close), n)

        if ranges[i] < ranges[i + 1]:
            start = i + 1
            end = min(n, start + samples)
            for j in range(start, end):
                ranges[j] = min(ranges[j], r_close)
        else:
            end = i + 1
            start = max(0, end - samples)
            for j in range(start, end):
                ranges[j] = min(ranges[j], r_close)

def find_best_gap(ranges, start_idx, end_idx):
    """
    Among the forward-looking beams, pick the gap with the furthest midpoint.
    Break ties on gap length, and fall back to the single furthest beam.
    """
    n = len(ranges)
    start_idx = max(0, min(n, start_idx))
    end_idx = max(start_idx, min(n, end_idx))

    safety_radius = (car_width / 2.0) + safety_margin
    best_gap_idx = None
    best_gap_dist = -float('inf')
    best_gap_len = 0
    curr_start = None
    max_dist_idx = None
    max_dist_val = -float('inf')

    def consider_gap(s_idx, e_idx, best_idx, best_dist, best_len):
        if s_idx is None or e_idx is None or e_idx <= s_idx:
            return best_idx, best_dist, best_len
        mid_idx = s_idx + (e_idx - s_idx) // 2
        if mid_idx < start_idx or mid_idx >= end_idx:
            return best_idx, best_dist, best_len
        dist_mid = ranges[mid_idx]
        length = e_idx - s_idx
        if (
            dist_mid > best_dist
            or (abs(dist_mid - best_dist) <= 1e-3 and length > best_len)
        ):
            return mid_idx, dist_mid, length
        return best_idx, best_dist, best_len

    for idx in range(start_idx, end_idx):
        dist = ranges[idx]
        if dist > max_dist_val:
            max_dist_val = dist
            max_dist_idx = idx

        if dist >= safety_radius:
            if curr_start is None:
                curr_start = idx
        else:
            best_gap_idx, best_gap_dist, best_gap_len = consider_gap(
                curr_start, idx, best_gap_idx, best_gap_dist, best_gap_len
            )
            curr_start = None

    best_gap_idx, best_gap_dist, best_gap_len = consider_gap(
        curr_start, end_idx, best_gap_idx, best_gap_dist, best_gap_len
    )

    if best_gap_idx is not None:
        return best_gap_idx
    if max_dist_idx is not None:
        return max_dist_idx
    return max(0, min(n - 1, start_idx))

def callback(data):
    ranges, min_range, max_range = sanitize_ranges(data)
    n = len(ranges)

    start_idx = angle_to_index(data, -math.pi / 2.0)
    end_idx = angle_to_index(data,  math.pi / 2.0)
    if start_idx > end_idx:
        start_idx, end_idx = end_idx, start_idx
    start_idx = max(0, start_idx)
    end_idx = min(n, end_idx)
    if end_idx <= start_idx:
        start_idx = 0
        end_idx = n

    disparities = []
    for i in range(max(0, start_idx - 1), min(n - 1, end_idx)):
        if abs(ranges[i + 1] - ranges[i]) > disp_threshold:
            disparities.append(i)

    extend_disparities(ranges, data.angle_increment, disparities, min_range)

    target_idx = find_best_gap(ranges, start_idx, end_idx)
    best_angle = data.angle_min + target_idx * data.angle_increment
    best_distance = ranges[target_idx]

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
