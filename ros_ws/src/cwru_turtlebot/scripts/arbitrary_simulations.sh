#!/usr/bin/env bash

NUM=$1

roslaunch cwru_turtlebot world_common_generic.launch number_of_robots:=$NUM gui:=true sim_time:=480&
sleep 1

for i in `seq 1 $NUM`;
do
    NAME="turtlebot$i"
    echo "Starting $NAME"
    roslaunch cwru_turtlebot one_mobile_generic.launch name:=$NAME x:=$i y:=$i &
done