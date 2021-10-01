#!/bin/bash

file_list="/media/kuriatsu/Samsung_TC2019/carla_drive_data/*time.pickle"
out_path="/media/kuriatsu/Samsung_TC2019/ros_drive_data/"

for read_file in ${file_list[@]}; do

    buf=${read_file##*/}
    out_file_name=${buf%time.pickle}
    echo 0 > last_int_distance.txt
    status=1

    while [ $status -ne 0 ]; do
        echo $status
        int_start_dist=$(cat last_int_distance.txt)
        out_file=$out_path$out_file_name"int_"$int_start_dist".pickle"
        echo $out_file

        python detect_collision.py &
        collision_ps=$!

        python intervention.py 1 &
        intervention_ps=$!

        python save_ros_data.py $out_file &
        save_data_ps=$!

        python play_carla_data.py $read_file &
        play_data_ps=$!

        wait $play_data_ps

        kill -2 $intervention_ps $save_data_ps $collision_ps

        sleep 5
        python check_file.py $out_file
        status=$?
        echo "status: " $status
    done

    status=1

    while [ $status -ne 0 ]; do
        out_file=$out_path$out_file_name"noint.pickle"
        echo $out_file

        python detect_collision.py &
        collision_ps=$!

        python intervention.py 0 &
        intervention_ps=$!

        python save_ros_data.py $out_file &
        save_data_ps=$!

        python play_carla_data.py $read_file &
        play_data_ps=$!

        wait $play_data_ps

        kill -2 $intervention_ps $save_data_ps $collision_ps

        sleep 5
        python check_file.py $out_file
        status=$?
        echo "status: " $status
    done
done
