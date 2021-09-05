#!/bin/bash
save_dir="/media/kuriatsu/Samsung_TC2019/carla_drive_data"
play_list=(
	"Town01,town01_vacant,20,60,2"
	"Town01,town01_crowd,40,100,2"
	"Town02,town02_vacant,20,60,2"
	"Town02,town02_crowd,40,100,2"
	"Town03,town03_vacant,60,80,2"
	"Town03,town03_crowd,100,120,2"
	"Town04,town04_vacant,60,80,2"
	"Town04,town04_crowd,100,120,2"
	"Town05,town05_vacant,60,80,2"
	"Town05,town05_crowd,100,120,2"
	"Town06,town06_vacant,60,80,2"
	"Town06,town06_crowd,100,120,2"
	"Town07,town07_vacant,20,60,2"
	"Town07,town07_crowd,40,100,2"
	)

# play rosbag & start aidi
for play_data in ${play_list[@]}; do

	map=$(echo $play_data | awk -F '[,]' '{print $1}')
    file_name=$(echo $play_data | awk -F '[,]' '{print $2}')
	vehicle=$(echo $play_data | awk -F '[,]' '{print $3}')
	walker=$(echo $play_data | awk -F '[,]' '{print $4}')
	itr=$(echo $play_data | awk -F '[,]' '{print $5}')
    echo map=$map itr=$itr

    out_num=0
    while [ $out_num -lt $itr ]; do
        start=$out_num
        python config.py -m $map
        sleep 1
        echo spawning npc
        python spawn_npc.py -n $vehicle -w $walker &
        spawn_np_ps=$!
        sleep 5
        echo start logging
        python save_carla_data.py $save_dir/$file_name $start $itr
        kill -2 $spawn_np_ps
        out_num=$(find $save_dir -name "*$file_name*.pickle" | wc -l)
    done


done

killall CarlaUE4-Linux-Shipping
