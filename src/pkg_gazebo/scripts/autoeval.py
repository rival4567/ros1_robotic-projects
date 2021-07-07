#!/usr/bin/env python3
import glob
import rosbag
import math
import csv

# USAGE: rosrun pkg_gazebo autoeval.py
# ROS Topics required: /gazebo/link_states, /depth_camera/depth/points


def evaluate_score(bag):
    '''This method evaulates scores by looking into bag files. It then
    returns strings showing outcome.'''

    topics = bag.get_type_and_topic_info()[1].keys()
    if '/turtle1/pose' and '/turtle2/pose' and '/turtle1/cmd_vel' and '/turtle2/cmd_vel' in topics:
        is_topic_present = True
    else:
        is_topic_present = False

    t1_pos_msg_list = []
    t2_pos_msg_list = []

    for topic, msg, t in bag.read_messages(topics=['/turtle1/pose']):
        t1_pos_msg_list.append((t, msg))
    for topic, msg, t in bag.read_messages(topics=['/turtle2/pose']):
        t2_pos_msg_list.append((t, msg))

    dist_list = merge_list_with_calculated_distance(
        t1_pos_msg_list, t2_pos_msg_list)

    t1_cmd_vel_list = []
    t2_cmd_vel_list = []
    for topic, msg, t in bag.read_messages(topics=['/turtle1/cmd_vel']):
        t1_cmd_vel_list.append((t, msg))
    for topic, msg, t in bag.read_messages(topics=['/turtle2/cmd_vel']):
        t2_cmd_vel_list.append((t, msg))

    is_turtle2_running = is_turtle2_moving_when_turtle1_moves(
        t1_cmd_vel_list, t2_pos_msg_list)

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

    if dist_list[idx - 1][1] < 0.05:
        is_reach_turtle1_spawn = True
    else:
        is_reach_turtle1_spawn = False

    if dist_list[-1][1] < 0.05 and percent_of_distance_increasing < 5 and is_turtle2_running:
        is_turtle2_following = True
    else:
        is_turtle2_following = False

    bag.close()
    return is_topic_present, is_reach_turtle1_spawn, is_turtle2_following


def main():
    bag_files = []
    # Change this path to where all bag files are located.
    path = "/home/ubuntu/workspace/src/pkg_gazebo/bag_files"
    for file in glob.glob(path + "/*.bag"):
        bag_files.append(file)

    print(bag_files)
    with open(path + '/' + 'results.csv', mode='w') as data_file:
        data_writer = csv.writer(data_file, delimiter=',',
                                 quotechar='"', quoting=csv.QUOTE_MINIMAL)

        data_writer.writerow(['filename', 'Is turtle2 spawned?',
                             'Does turtle2 reach turtle1 spawn position?', 'Is turtle2 following turtle1?', 'Remark', 'Is Manual Evaluation required if a query is raised?'])
        for file in bag_files:
            bag = rosbag.Bag(file)
            score = evaluate_score(bag)
            remark = get_remark(score)
            data_writer.writerow(
                [file.split('/')[-1].split('.')[-2], score[0], score[1], score[2], remark[0], remark[1]])

    print("Finished creating csv file!")
