#!/usr/bin/env bash

mkdir -p /home/matt/thesis/experiment_data/one_stationary/
roslaunch cwru_turtlebot one_stationary.launch save_file_prefix:="/home/matt/thesis/experiment_data/one_stationary/" gui:=false sim_time:=120

mkdir -p /home/matt/thesis/experiment_data/one_mobile/
roslaunch cwru_turtlebot one_mobile.launch save_file_prefix:="/home/matt/thesis/experiment_data/one_mobile/" gui:=false sim_time:=120
