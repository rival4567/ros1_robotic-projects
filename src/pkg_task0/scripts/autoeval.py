#!/usr/bin/env python3
import yaml
import sys
import os
import rosbag
import math

# USAGE EXAMPLE:
# rosrun pkg_task0 autoeval.py <path-to-bag-files> <file-name>

filename = sys.argv[2]
directory = sys.argv[1]
print("Reading the rosbag file")
if not directory.endswith("/"):
    directory += "/"
extension = ""
if not filename.endswith(".bag"):
    extension = ".bag"
bag = rosbag.Bag(directory + filename + extension)

# Create directory with name filename (without extension)
results_dir = directory + filename[:-4] + "_results"
if not os.path.exists(results_dir):
    os.makedirs(results_dir)

t1_pos_msg_list = []
t2_pos_msg_list = []

for topic, msg, t in bag.read_messages(topics=['/turtle1/pose']):
    t1_pos_msg_list.append((t, msg))
for topic, msg, t in bag.read_messages(topics=['/turtle2/pose']):
    t2_pos_msg_list.append((t, msg))


def calculate_distance(x0, y0, x1, y1):
    '''Calculate distance between two points in 2D. Input points (x0, y0) and
    (x1, y1).'''
    return math.sqrt((x0 - x1)**2 + (y0 - y1)**2)


def merge_list_with_calculated_distance(list1, list2):
    '''Takes turtle1 and turtl2 position list with timestamps as arguments and
    return a list of distance between turtle1 and turtle2 with timestamps.'''
    l1 = [(t, msg, 1) for (t, msg) in list1]
    l2 = [(t, msg, 2) for (t, msg) in list2]
    new_list = l1 + l2
    # Sort list based on timestamps and index provided for t1 and t2 as 1, 2
    sorted_list = sorted(
        new_list, key=lambda element: (element[0], element[2]))
    c1 = list1[0][1]  # first value of msg
    c2 = list2[0][1]
    last_time = -1
    result = []
    for t, msg, index in sorted_list:
        if index == 1:
            c1 = msg
        elif index == 2:
            c2 = msg
        elif t == last_time:
            result.pop()
        result.append((t, calculate_distance(c1.x, c1.y, c2.x, c2.y)))
    return result


dist_list = merge_list_with_calculated_distance(
    t1_pos_msg_list, t2_pos_msg_list)

t1_cmd_vel_list = []
for topic, msg, t in bag.read_messages(topics=['/turtle1/cmd_vel']):
    t1_cmd_vel_list.append((t, msg))

# Get the first publish timestamp of /turtle1/cmd_vel
first_pub = t1_cmd_vel_list[0][0]

# Find the closest value in the dist_list timestamp
before_mov_t1 = min(dist_list, key=lambda list: abs(list[0] - first_pub))
idx = dist_list.index(before_mov_t1)

i = 1   # To ignore first element starting from 1 instead of 0
count = 0   # Count the number of time distance is increasing.
while i < idx:
    if dist_list[i][1] > dist_list[i-1][1]:
        count += 1
    i += 1

percent_of_distance_increasing = (count * 100) / idx

if dist_list[idx - 1][1] <= 0.05:
    print("Successfully reached turtle1 at spawn position.")

if dist_list[-1][1] < 0.14 and percent_of_distance_increasing < 5:
    print("Successfully following turtle1.")

bag.close()
