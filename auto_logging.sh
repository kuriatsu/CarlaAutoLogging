#!/bin/bash

play_list=(
	"Town01,town01_vacant,20,40,100"
	"Town01,town01_crowd,40,80,100"
	"Town02,town02_vacant,20,40,100"
	"Town02,town02_crowd,40,80,100"
	"Town03,town02_vacant,20,40,100"
	"Town03,town02_crowd,40,80,100"
	"Town04,town02_vacant,20,40,100"
	"Town04,town02_crowd,40,80,100"
	"Town05,town02_vacant,20,40,100"
	"Town05,town02_crowd,40,80,100"
	"Town06,town02_vacant,20,40,100"
	"Town06,town02_crowd,40,80,100"
	"Town07,town02_vacant,20,40,100"
	"Town07,town02_crowd,40,80,100"
	)

# play rosbag & start aidi
for play_data in ${play_list[@]}; do

	map=$(echo $play_data | awk -F '[,]' '{print $1}')
	log_name=$(echo $play_data | awk -F '[,]' '{print $2}')
	vehicle=$(echo $play_data | awk -F '[,]' '{print $3}')
	walker=$(echo $play_data | awk -F '[,]' '{print $4}')
	itr=$(echo $play_data | awk -F '[,]' '{print $5}')

	python config.py -m $map
    python spawn_npc.py -n $vehicle -w $walker &
    spawn_np_ps = $!
    python save_data.py $log_name $itr
    kill -2 $spawn_np_ps

done
