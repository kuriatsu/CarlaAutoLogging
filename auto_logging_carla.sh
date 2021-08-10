#!/bin/bash

play_list=(
	"Town01,town01_vacant,20,40,10"
	"Town01,town01_crowd,40,80,10"
	"Town02,town02_vacant,20,40,10"
	"Town02,town02_crowd,40,80,10"
	"Town03,town03_vacant,40,80,10"
	"Town03,town03_crowd,60,100,10"
	"Town04,town04_vacant,40,60,10"
	"Town04,town04_crowd,60,100,10"
	"Town05,town05_vacant,20,40,10"
	"Town05,town05_crowd,40,80,10"
	"Town06,town06_vacant,20,40,10"
	"Town06,town06_crowd,40,80,10"
	"Town07,town07_vacant,20,40,10"
	"Town07,town07_crowd,40,80,10"
	)

# play rosbag & start aidi
for play_data in ${play_list[@]}; do

	map=$(echo $play_data | awk -F '[,]' '{print $1}')
	log_name=$(echo $play_data | awk -F '[,]' '{print $2}')
	vehicle=$(echo $play_data | awk -F '[,]' '{print $3}')
	walker=$(echo $play_data | awk -F '[,]' '{print $4}')
	itr=$(echo $play_data | awk -F '[,]' '{print $5}')
    echo map=$map itr=$itr

	python config.py -m $map
    sleep 1
    echo spawning npc
    python spawn_npc.py -n $vehicle -w $walker &
    spawn_np_ps=$!
    sleep 1
    echo start logging
    python save_carla_data.py $log_name $itr
    kill -2 $spawn_np_ps

done
