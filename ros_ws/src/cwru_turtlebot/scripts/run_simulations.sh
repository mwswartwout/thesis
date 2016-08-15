#!/usr/bin/env bash

kill_everything() {
    echo Killing everything
    rosnode kill -a
    killall gzserver
    killall gzclient
    killall rosmaster
}

declare -i mobile_time=480
declare -i stationary_time=60
declare -i sleep_time=20

one_mobile=false
two_stationary=true
two_mobile=false

if [ "$one_mobile" = true ] ; then
    # mobile, noiseless, no gps
    roslaunch cwru_turtlebot one_robot.launch noisy:=false gps:=false stationary:=false sim_time:=${mobile_time}
    kill_everything
    sleep ${sleep_time}

    # mobile, noiseless, w/ gps
    roslaunch cwru_turtlebot one_robot.launch noisy:=false gps:=true stationary:=false sim_time:=${mobile_time}
    kill_everything
    sleep ${sleep_time}

    # mobile, noisy, no gps
    roslaunch cwru_turtlebot one_robot.launch noisy:=true gps:=false stationary:=false sim_time:=${mobile_time}
    kill_everything
    sleep ${sleep_time}

    # mobile, noisy, w/ gps
    roslaunch cwru_turtlebot one_robot.launch noisy:=true gps:=true stationary:=false sim_time:=${mobile_time}
    kill_everything
    sleep ${sleep_time}
fi

if [ "$two_stationary" = true ] ; then
    # two stationary, noiseless, no gps
    roslaunch cwru_turtlebot two_robot.launch noisy:=false gps:=false stationary:=true sim_time:=${stationary_time}
    kill_everything
    sleep ${sleep_time}

    # two stationary, noiseless, w/ gps
    roslaunch cwru_turtlebot two_robot.launch noisy:=false gps:=true stationary:=true sim_time:=${stationary_time}
    kill_everything
    sleep ${sleep_time}

    # two stationary, noisy, no gps
    roslaunch cwru_turtlebot two_robot.launch noisy:=true gps:=false stationary:=true sim_time:=${stationary_time}
    kill_everything
    sleep ${sleep_time}

    # two stationary, noisy, w/ gps
    roslaunch cwru_turtlebot two_robot.launch noisy:=true gps:=true stationary:=true sim_time:=${stationary_time}
    kill_everything
    sleep ${sleep_time}
fi

if [ "$two_mobile" = true ] ; then
    # two mobile, noiseless, no gps
    roslaunch cwru_turtlebot two_robot.launch noisy:=false gps:=false stationary:=false sim_time:=${mobile_time}
    kill_everything
    sleep ${sleep_time}

    # two mobile, noiseless, w/ gps
    roslaunch cwru_turtlebot two_robot.launch noisy:=false gps:=true stationary:=false sim_time:=${mobile_time}
    kill_everything
    sleep ${sleep_time}

    # two mobile, noisy, no gps
    roslaunch cwru_turtlebot two_robot.launch noisy:=true gps:=false stationary:=false sim_time:=${mobile_time}
    kill_everything
    sleep ${sleep_time}

    # two mobile, noisy, w/ gps
    roslaunch cwru_turtlebot two_robot.launch noisy:=true gps:=true stationary:=false sim_time:=${mobile_time}
    kill_everything
    sleep ${sleep_time}
fi

(cd ~/thesis/analysis && ./analyze_data.R)

