#!/bin/bash

file_list="/media/kuriatsu/Samsung_TC2019/ros_drive_data/*.pickle"
out_path="/media/kuriatsu/Samsung_TC2019/extracted_data/"

for read_file in ${file_list[@]}; do
    noint_file=${read_file%int*.pickle}"noint.pickle"
    int_out_file=$out_path${read_file##*/}
    noint_out_file=$out_path$(sed -e 's/int/noint/' <<< $int_out_file)
    echo $read_file #$noint_file_name $int_out_file $noint_out_file

    python post_treatment.py $read_file $noint_file
done
