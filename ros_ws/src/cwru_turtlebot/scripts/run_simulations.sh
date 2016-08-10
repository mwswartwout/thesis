#!/usr/bin/env bash

kill_everything() {
    echo Killing everything
    rosnode kill -a
    killall gzserver
    killall gzclient
    killall rosmaster
}

declare -i time=1800

# stationary, noiseless, no gps
roslaunch cwru_turtlebot one_robot.launch noisy:=false gps:=false stationary:=true sim_time:=${time}
kill_everything

# stationary, noiseless, w/ gps
roslaunch cwru_turtlebot one_robot.launch noisy:=false gps:=true stationary:=true sim_time:=${time}
kill_everything

# mobile, noiseless, no gps
roslaunch cwru_turtlebot one_robot.launch noisy:=false gps:=false stationary:=false sim_time:=${time}
kill_everything

# mobile, noiseless, w/ gps
roslaunch cwru_turtlebot one_robot.launch noisy:=false gps:=true stationary:=false sim_time:=${time}
kill_everything

# stationary, noisy, no gps
roslaunch cwru_turtlebot one_robot.launch noisy:=true gps:=false stationary:=true sim_time:=${time}
kill_everything

# stationary, noisy, w/ gps
roslaunch cwru_turtlebot one_robot.launch noisy:=true gps:=true stationary:=true sim_time:=${time}
kill_everything

# mobile, noisy, no gps
roslaunch cwru_turtlebot one_robot.launch noisy:=true gps:=false stationary:=false sim_time:=${time}
kill_everything

# mobile, noisy, w/ gps
roslaunch cwru_turtlebot one_robot.launch noisy:=true gps:=true stationary:=false sim_time:=${time}
kill_everything

# two stationary, noiseless, no gps
roslaunch cwru_turtlebot two_robot.launch noisy:=false gps:=false stationary:=true sim_time:=${time}
kill_everything

# two stationary, noiseless, w/ gps
roslaunch cwru_turtlebot two_robot.launch noisy:=false gps:=true stationary:=true sim_time:=${time}
kill_everything

# two stationary, noisy, no gps
roslaunch cwru_turtlebot two_robot.launch noisy:=true gps:=false stationary:=true sim_time:=${time}
kill_everything

# two stationary, noisy, w/ gps
roslaunch cwru_turtlebot two_robot.launch noisy:=true gps:=true stationary:=true sim_time:=${time}
kill_everything

# two mobile, noiseless, no gps
roslaunch cwru_turtlebot two_robot.launch noisy:=false gps:=false stationary:=false sim_time:=${time}
kill_everything

# two mobile, noiseless, w/ gps
roslaunch cwru_turtlebot two_robot.launch noisy:=false gps:=true stationary:=false sim_time:=${time}
kill_everything

# two mobile, noisy, no gps
roslaunch cwru_turtlebot two_robot.launch noisy:=true gps:=false stationary:=false sim_time:=${time}
kill_everything

# two mobile, noisy, w/ gps
roslaunch cwru_turtlebot two_robot.launch noisy:=true gps:=true stationary:=false sim_time:=${time}
kill_everything

(cd ~/thesis/analysis && ./analyze_data.R)
