#!/bin/bash

file_list="/media/kuriatsu/Samsung_TC2019/carla_drive_data/*time.pickle"
out_path="/media/kuriatsu/Samsung_TC2019/ros_simulate_data/"

intervention_list=("no_int" "int")
# source /home/kuriatsu/Source/autoware-1.13/install/setup.bash
# roslaunch autoware.launch &
# autoware_ps=$!

for read_file in ${file_list[@]}; do

    for mode in ${intervention_list[@]}; do
        buf=${read_file##*/}
        out_file_name=${buf%dist.pickle}
        out_file=$out_path$out_file_name$mode".pickle"

        echo $out_file

        python detect_collision.py &
        collision_ps=$!

        if [ $mode = "int" ]; then
            is_intervention=1
        else
            is_intervention=0
        fi

        python intervention.py $is_intervention &
        intervention_ps=$!

        python play_data_time.py $read_file &
        play_data_ps=$!

        python save_ros_data_time.py $read_file $out_file &
        save_data_ps=$!

        wait $play_data_ps

        kill -2 $intervention_ps $save_data_ps $collision_ps

    done
done
