#!/bin/bash

file_list="/media/kuriatsu/Samsung_TC2019/ros_drive_data/*.pickle"

roscore &
roscore_ps=&!
wait 5

rviz -d play_ros_data.rviz &
rviz_ps=$!
wait 5
for read_file in ${file_list[@]}; do
    echo $read_file
    python play_ros_data.py $read_file

done

kill -2 $roscore_ps
kill -2 $rviz_ps
