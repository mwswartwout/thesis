#!/usr/bin/env bash

#roslaunch cwru_turtlebot one_stationary.launch sim_time:=480

roslaunch cwru_turtlebot one_mobile.launch sim_time:=480

roslaunch cwru_turtlebot two_stationary.launch sim_time:=480

roslaunch cwru_turtlebot two_mobile.launch sim_time:=480

#roslaunch cwru_turtlebot two_mobile_restricted.launch save_file_prefix:="/home/matt/thesis/experiment_data/two_mobile_restricted/" gui:=false sim_time:=7200

#roslaunch cwru_turtlebot five_mobile.launch save_file_prefix:="/home/matt/thesis/experiment_data/five_mobile/" gui:=false sim_time:=7200

#roslaunch cwru_turtlebot five_mobile_restricted.launch save_file_prefix:="/home/matt/thesis/experiment_data/five_mobile_restricted/" gui:=false sim_time:=7200
