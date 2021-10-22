#!/bin/bash
save_dir="/media/kuriatsu/Samsung_TC2019/carla_drive_data"
play_list=(
    "Town01,town01_vacant,20,60,20,30"
	"Town01,town01_vacant,20,60,30,40"
    "Town01,town01_crowd,40,100,20,30"
	"Town01,town01_crowd,40,100,30,40"
    "Town02,town02_vacant,20,60,20,30"
	"Town02,town02_vacant,20,60,30,40"
    "Town02,town02_crowd,40,100,20,30"
	"Town02,town02_crowd,40,100,30,40"
    "Town03,town03_vacant,60,80,20,30"
	"Town03,town03_vacant,60,80,30,40"
    "Town03,town03_crowd,100,120,20,30"
	"Town03,town03_crowd,100,120,30,40"
    "Town04,town04_vacant,60,60,20,30"
	"Town04,town04_vacant,60,60,30,40"
    "Town04,town04_crowd,100,100,20,30"
	"Town04,town04_crowd,100,100,30,40"
    "Town05,town05_vacant,60,80,20,30"
	"Town05,town05_vacant,60,80,30,40"
    "Town05,town05_crowd,100,120,20,30"
	"Town05,town05_crowd,100,120,30,40"
    "Town06,town06_vacant,60,80,20,30"
	"Town06,town06_vacant,60,80,30,40"
    "Town06,town06_crowd,100,120,20,30"
	"Town06,town06_crowd,100,120,30,40"
    "Town07,town07_vacant,20,60,20,30"
	"Town07,town07_vacant,20,60,30,40"
    "Town07,town07_crowd,40,100,20,30"
	"Town07,town07_crowd,40,100,30,40"
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

        max="${save_dir}/${file_name}_${start}_time.pickle"
        for file_name in $(find $save_dir -name "*$file_name*_time.pickle"); do
            if [[ "${save_dir}/${file_name}_${start}_time.pickle" < $file_name ]]; then
                if [[ $file_name < "${save_dir}/${file_name}_${end}_time.pickle" ]]; then
                    if [[ $file_name > $max ]]; then
                        max=$file_name
                    fi
                fi
            fi
        done
        buf=${max%_*}
        start=${buf##*_}
    done


done

killall CarlaUE4-Linux-Shipping
