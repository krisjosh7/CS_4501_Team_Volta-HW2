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


def find_best_gap(ranges, start_idx, end_idx):
    n = len(ranges)
    start_idx = max(0, min(n, start_idx))
    end_idx = max(start_idx + 1, min(n, end_idx))

    safety_radius = (CAR_WIDTH / 2.0) + SAFETY_MARGIN
    best_idx = None
    best_dist = -float('inf')
    best_len = 0

    curr_start = None
    max_dist_idx = None
    max_dist_val = -float('inf')

    def consider_gap(s_idx, e_idx, best_idx, best_dist, best_len):
        if s_idx is None or e_idx <= s_idx:
            return best_idx, best_dist, best_len
        mid_idx = s_idx + (e_idx - s_idx) // 2
        dist_mid = ranges[mid_idx]
        gap_len = e_idx - s_idx
        if dist_mid > best_dist or (abs(dist_mid - best_dist) <= 1e-3 and gap_len > best_len):
            return mid_idx, dist_mid, gap_len
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
            best_idx, best_dist, best_len = consider_gap(curr_start, idx, best_idx, best_dist, best_len)
            curr_start = None

    best_idx, best_dist, best_len = consider_gap(curr_start, end_idx, best_idx, best_dist, best_len)

    if best_idx is not None:
        return best_idx
    if max_dist_idx is not None:
        return max_dist_idx
    return start_idx


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
