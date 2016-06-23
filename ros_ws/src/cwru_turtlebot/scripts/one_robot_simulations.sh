#!/usr/bin/env bash

roslaunch cwru_turtlebot one_stationary.launch save_file_prefix:="/home/matt/thesis/experiment_data/one_stationary/" gui:=false sim_time:=240 sensor_record:=true

roslaunch cwru_turtlebot one_mobile.launch save_file_prefix:="/home/matt/thesis/experiment_data/one_mobile/" gui:=false sim_time:=240 sensor_record:=true
