#!/usr/bin/env python

import math
import numpy
import csv

# Odometry model taken from Probabilistic Robotics by Thurn et al.
# Algorithm used is sample_motion_model_odometry from Table 5.6

# Robot specific noise parameters
# Default of 0.2 taken from the AMCL and MATLAB packages that implement this motion model
alpha_1 = .02
alpha_2 = .02
alpha_3 = .02
alpha_4 = .02

# Make our received odom reading more palatable
current_odom_x = 1
current_odom_y = 1
current_odom_theta = math.pi / 4

previous_odom_x = 0
previous_odom_y = 0
previous_odom_theta = 0

previous_pose_x = 0
previous_pose_y = 0
previous_pose_theta = 0

# Split movement into three distinct actions, rotate -> translate -> rotate
delta_rotation_1 = math.atan2(current_odom_y - previous_odom_y, current_odom_x - previous_odom_x) - previous_odom_theta
delta_translation = math.sqrt((previous_odom_x - current_odom_x) ** 2 + (previous_odom_y - current_odom_y) ** 2)
delta_rotation_2 = current_odom_theta - previous_odom_theta - delta_rotation_1

std_dev_1 = alpha_1 * abs(delta_rotation_1) + alpha_2 * delta_translation
std_dev_2 = alpha_3 * delta_translation + alpha_4 * (abs(delta_rotation_1) + abs(delta_rotation_2))
std_dev_3 = alpha_1 * abs(delta_rotation_2) + alpha_2 * delta_translation

repetitions = 500

noise_1 = numpy.random.normal(scale=std_dev_1, size=repetitions)
noise_2 = numpy.random.normal(scale=std_dev_2, size=repetitions)
noise_3 = numpy.random.normal(scale=std_dev_3, size=repetitions)

filename = 'odom_noise_test.csv'
with open(filename, 'w+') as csv_file:
    writer = csv.writer(csv_file)
    writer.writerow(['x', 'y', 'theta'])

    for i in range(0, repetitions):
        delta_rotation_1_hat = delta_rotation_1 - noise_1[i]
        delta_translation_hat = delta_translation - noise_2[i]
        delta_rotation_2_hat = delta_rotation_2 - noise_3[i]

        noisy_odom_x = previous_pose_x + delta_translation_hat * math.cos(previous_pose_theta + delta_rotation_1_hat)
        noisy_odom_y = previous_pose_y + delta_translation_hat * math.sin(previous_pose_theta + delta_rotation_1_hat)
        noisy_odom_yaw = previous_pose_theta + delta_rotation_1_hat + delta_rotation_2_hat

        writer.writerow([noisy_odom_x, noisy_odom_y, noisy_odom_yaw])
