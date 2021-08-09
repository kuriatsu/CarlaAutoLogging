#!/bin/bash

file_list="/home/kuriatsu/Source/CarlaAutoLogging/carla_data/*"
out_path="/home/kuriatsu/Source/CarlaAutoLogging/ros_data/"

roslaunch autoware.launch
autoware_ps=$!

for read_file in ${file_list[@]}; do
    if [ -f ${read_file}]; then
        continue
    fi

    for mode in ["no_int" "int"]; do
        buf=${read_file##*/}
        out_file_name=${buf%.pickle}
        out_file=$out_path$out_file_name"_"$mode".pickle"

        python detect_collision.py &
        collision_ps=$!

        python play_data.py $read_file &
        play_data_ps=$!

        sleep 1.0

        python auto_intervention.py $read_file 1 &
        intervention_ps=$!

        python save_ros_data.py $read_file $out_file &
        save_data_ps=$!
        
        wait $play_data_ps

        kill -2 $intervention_ps $save_data_ps $collision_ps
