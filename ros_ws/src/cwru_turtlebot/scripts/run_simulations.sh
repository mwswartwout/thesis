#!/usr/bin/env bash

kill_everything() {
    echo Killing everything
    rosnode kill -a
    killall gzserver
    killall rosmaster
}

# First run noiseless experiments
roslaunch cwru_turtlebot one_stationary.launch sim_time:=500
kill_everything

roslaunch cwru_turtlebot one_mobile.launch sim_time:=500
kill_everything

roslaunch cwru_turtlebot two_stationary.launch sim_time:=500
kill_everything

roslaunch cwru_turtlebot two_mobile.launch sim_time:=500
kill_everything

roslaunch cwru_turtlebot two_mobile_restricted.launch sim_time:=500
kill_everything

roslaunch cwru_turtlebot five_mobile.launch sim_time:=500
kill_everything

roslaunch cwru_turtlebot five_mobile_restricted.launch sim_time:=500
kill_everything

# Now run noisy experiments
roslaunch cwru_turtlebot one_stationary.launch sim_time:=500 noisy:=true
kill_everything

roslaunch cwru_turtlebot one_mobile.launch sim_time:=500 noisy:=true
kill_everything

roslaunch cwru_turtlebot two_stationary.launch sim_time:=500 noisy:=true
kill_everything

roslaunch cwru_turtlebot two_mobile.launch sim_time:=500 noisy:=true
kill_everything

roslaunch cwru_turtlebot two_mobile_restricted.launch sim_time:=500 noisy:=true
kill_everything

roslaunch cwru_turtlebot five_mobile.launch sim_time:=500
kill_everything

roslaunch cwru_turtlebot five_mobile_restricted.launch sim_time:=500
kill_everything