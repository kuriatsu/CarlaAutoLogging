#!/bin/bash
save_dir="/media/kuriatsu/Samsung_TC2019/carla_drive_data"
play_list=(
    "Town01,town01_vacant,30,40,60,70"
	"Town01,town01_vacant,30,40,70,80"
    "Town01,town01_crowd,50,80,60,70"
	"Town01,town01_crowd,50,80,70,80"
    "Town01,town01_vacant,30,40,80,90"
	"Town01,town01_vacant,30,40,90,100"
    "Town01,town01_crowd,50,80,80,90"
	"Town01,town01_crowd,50,80,90,100"
    "Town02,town02_vacant,30,40,60,70"
	"Town02,town02_vacant,30,40,70,80"
    "Town02,town02_crowd,50,80,60,70"
	"Town02,town02_crowd,50,80,70,80"
    "Town02,town02_vacant,30,40,80,90"
	"Town02,town02_vacant,30,40,90,100"
    "Town02,town02_crowd,50,80,80,90"
	"Town02,town02_crowd,50,80,90,100"
    "Town03,town03_vacant,50,70,60,70"
	"Town03,town03_vacant,50,70,70,80"
    "Town03,town03_crowd,70,90,60,70"
	"Town03,town03_crowd,70,90,70,80"
    "Town03,town03_vacant,50,70,80,90"
	"Town03,town03_vacant,50,70,90,100"
    "Town03,town03_crowd,70,90,80,90"
	"Town03,town03_crowd,70,90,90,100"
    "Town04,town04_vacant,30,50,60,70"
	"Town04,town04_vacant,30,50,70,80"
    "Town04,town04_crowd,50,80,60,70"
	"Town04,town04_crowd,50,80,70,80"
    "Town04,town04_vacant,30,50,80,90"
	"Town04,town04_vacant,30,50,90,100"
    "Town04,town04_crowd,50,80,80,90"
	"Town04,town04_crowd,50,80,90,100"
    "Town05,town05_vacant,50,70,60,70"
	"Town05,town05_vacant,50,70,70,80"
    "Town05,town05_crowd,80,80,60,70"
	"Town05,town05_crowd,80,80,70,80"
    "Town05,town05_vacant,50,70,80,90"
	"Town05,town05_vacant,50,70,90,100"
    "Town05,town05_crowd,80,80,80,90"
	"Town05,town05_crowd,80,80,90,100"
    "Town06,town06_vacant,40,60,60,70"
	"Town06,town06_vacant,40,60,70,80"
    "Town06,town06_crowd,80,80,60,70"
	"Town06,town06_crowd,80,80,70,80"
    "Town06,town06_vacant,40,60,80,90"
	"Town06,town06_vacant,40,60,90,100"
    "Town06,town06_crowd,80,80,80,90"
	"Town06,town06_crowd,80,80,90,100"
    "Town07,town07_vacant,20,50,60,70"
	"Town07,town07_vacant,20,50,70,80"
    "Town07,town07_crowd,40,70,60,70"
	"Town07,town07_crowd,40,70,70,80"
    "Town07,town07_vacant,20,50,80,90"
	"Town07,town07_vacant,20,50,90,100"
    "Town07,town07_crowd,40,70,80,90"
	"Town07,town07_crowd,40,70,90,100"
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

    while [ $start_num -lt $(( $end_num-1 )) ]; do
        python ../carla_tools/config.py -m $map
        sleep 1
        echo spawning npc
        python ../carla_tools/spawn_npc.py -n $vehicle -w $walker &
        spawn_np_ps=$!
        sleep 5
        echo start_num logging
        python save_carla_data.py $save_dir/$file_name $start_num $end_num
        kill -2 $spawn_np_ps

        max="${save_dir}/${file_name}_${start_num}_time.pickle"
        for itr in $(find $save_dir -name "*$file_name*_time.pickle"); do
            if [[ "${save_dir}/${file_name}_${start_num}_time.pickle" < $itr ]]; then
                if [[ $itr < "${save_dir}/${file_name}_${end}_time.pickle" ]]; then
                    if [[ $itr > $max ]]; then
                        max=$itr
                        echo "max" $max
                    fi
                fi
            fi
        done
        buf=${max%_*}
        start_num=${buf##*_}
        echo "buf start = " $buf $start_num
    done


done

# killall CarlaUE4-Linux-Shipping
