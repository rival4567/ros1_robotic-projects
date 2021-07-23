#!/usr/bin/env python3
import glob
import rosbag
import math
import csv

# USAGE: rosrun micromouse autoeval.py
# This file will print results on the terminal window

def remap(t):
	d = 0.012  # Wall thickness in m
	s = 0.18-d  # Side length of each grid square sans the walls
	N = 16  # Number of grid squares in one row
	return t/(s+d) + N/2

def main():
	bag_files = []
	is_passed = False
	is_topic_present = False
	dest_reached = False
	collision = False
	# Change this path to where all bag files are located.
	path = "/home/ubuntu/workspace/src/micromouse/bag_files"
	for file in glob.glob(path + "/*.bag"):
		bag_files.append(file)

	# iterate through each file
	for file in bag_files:
		bag = rosbag.Bag(file)
		# look throuhg topics
		topics = bag.get_type_and_topic_info()[1].keys()
		if '/micromouse/odom' in topics:
			is_topic_present = True
		else:
			is_topic_present = False
			print('topic not present')
			is_passed = False

		for topic, msg, t in bag.read_messages(topics=['/micromouse/odom']):
			# check if destination reached
			x_bot = msg.pose.pose.position.x
			y_bot = msg.pose.pose.position.y
			x_bot_remapped = remap(x_bot)
			y_bot_remapped = remap(y_bot)
			
			# Coordinates in 31*31 maze_state representation
			X = int(x_bot_remapped)*2
			Y = int(y_bot_remapped)*2

		for topic, msg, t in bag.read_messages(topics=['/micromouse/laser/scan']):
			# check collision
			laser_ranges = msg.ranges
			no_of_points = len(laser_ranges)
			for i in range(no_of_points):
				if laser_ranges[i] < 0.05: #assuming distance less than 0.05 will result in collision
					collision = True

		if X == 14 and Y == 14:
			dest_reached = True

		if dest_reached == True and collision == False:
			is_passed = True

		if is_topic_present == True and is_passed == True:
			print(file + ' Result: passed!')
		else:
			print(file + ' Result: failed!')

if __name__ == '__main__':
	main()