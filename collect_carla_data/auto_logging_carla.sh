#!/bin/bash
save_dir="/media/kuriatsu/Samsung_TC2019/carla_drive_data"
play_list=(
    "Town01,town01_vacant,20,60,0,10"
	"Town01,town01_vacant,20,60,10,20"
    "Town01,town01_crowd,40,100,0,10"
	"Town01,town01_crowd,40,100,10,20"
    "Town02,town02_vacant,20,60,0,10"
	"Town02,town02_vacant,20,60,10,20"
    "Town02,town02_crowd,40,100,0,10"
	"Town02,town02_crowd,40,100,10,20"
    "Town03,town03_vacant,60,80,0,10"
	"Town03,town03_vacant,60,80,10,20"
    "Town03,town03_crowd,100,120,0,10"
	"Town03,town03_crowd,100,120,10,20"
    "Town04,town04_vacant,60,60,0,10"
	"Town04,town04_vacant,60,60,10,20"
    "Town04,town04_crowd,100,100,0,10"
	"Town04,town04_crowd,100,100,10,20"
    "Town05,town05_vacant,60,80,0,10"
	"Town05,town05_vacant,60,80,10,20"
    "Town05,town05_crowd,100,120,0,10"
	"Town05,town05_crowd,100,120,10,20"
    "Town06,town06_vacant,60,80,0,10"
	"Town06,town06_vacant,60,80,10,20"
    "Town06,town06_crowd,100,120,0,10"
	"Town06,town06_crowd,100,120,10,20"
    "Town07,town07_vacant,20,60,0,10"
	"Town07,town07_vacant,20,60,10,20"
    "Town07,town07_crowd,40,100,0,10"
	"Town07,town07_crowd,40,100,10,20"
	)

# play rosbag & start aidi
for play_data in ${play_list[@]}; do

	map=$(echo $play_data | awk -F '[,]' '{print $1}')
    file_name=$(echo $play_data | awk -F '[,]' '{print $2}')
	vehicle=$(echo $play_data | awk -F '[,]' '{print $3}')
	walker=$(echo $play_data | awk -F '[,]' '{print $4}')
    start_num=$(echo $play_data | awk -F '[,]' '{print $5}')
	end_num=$(echo $play_data | awk -F '[,]' '{print $6}')
    echo map=$map itr=$end_num

    while [ $start_num -lt $end_num ]; do
        start=$start_num
        python ../carla_tools/config.py -m $map
        sleep 1
        echo spawning npc
        python ../carla_tools/spawn_npc.py -n $vehicle -w $walker &
        spawn_np_ps=$!
        sleep 5
        echo start logging
        python save_carla_data.py $save_dir/$file_name $start $end_num
        kill -2 $spawn_np_ps
        start_num=$(find $save_dir -name "*$file_name*.pickle" | wc -l)
    done


done

killall CarlaUE4-Linux-Shipping
