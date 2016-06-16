#!/usr/bin/env bash

mkdir -p /home/matt/thesis/experiment_data/one_stationary/
roslaunch cwru_turtlebot one_stationary.launch save_file_prefix:="/home/matt/thesis/experiment_data/one_stationary/" gui:=false sim_time:=7200

mkdir -p /home/matt/thesis/experiment_data/one_mobile/
roslaunch cwru_turtlebot one_mobile.launch save_file_prefix:="/home/matt/thesis/experiment_data/one_mobile/" gui:=false sim_time:=7200

mkdir -p /home/matt/thesis/experiment_data/two_stationary/
roslaunch cwru_turtlebot two_stationary.launch save_file_prefix:="/home/matt/thesis/experiment_data/two_stationary/" gui:=false sim_time:=7200

mkdir -p /home/matt/thesis/experiment_data/two_mobile/
roslaunch cwru_turtlebot two_mobile.launch save_file_prefix:="/home/matt/thesis/experiment_data/two_mobile/" gui:=false sim_time:=7200

mkdir -p /home/matt/thesis/experiment_data/two_mobile_restricted/
roslaunch cwru_turtlebot two_mobile_restricted.launch save_file_prefix:="/home/matt/thesis/experiment_data/two_mobile_restricted/" gui:=false sim_time:=7200

mkdir -p /home/matt/thesis/experiment_data/five_mobile/
roslaunch cwru_turtlebot five_mobile.launch save_file_prefix:="/home/matt/thesis/experiment_data/five_mobile/" gui:=false sim_time:=7200

mkdir -p /home/matt/thesis/experiment_data/five_mobile_restricted/
roslaunch cwru_turtlebot five_mobile_restricted.launch save_file_prefix:="/home/matt/thesis/experiment_data/five_mobile_restricted/" gui:=false sim_time:=7200
