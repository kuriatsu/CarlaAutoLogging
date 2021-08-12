# CarlaAutoLogging

## Logging with CARLA
Collect data while driving with CARLA's automated driving function

1. Spawn agents (pedestrian, other vehicles)
2. Spawn ego_vehicle
3. Start driving and collect data for 30 seconds
4. Stop logging and remove ego_vehicle
5. Back to `2`

### Requirement
* CARLA (0.9.8)

### Setup
Configure `auto_logging_carla.sh`

* save_dir : Directory to store log file
* play_list : (`town name`, `filename prefix`, `vehicle number`, `pedestrian number`, `iteration` ...)
    * town name : carla town
    * filename prefix : log file will be ex. `/<save_dir>/<filename_prefix>_<iteration>.log`
    * vehicle number : Vehicle (bike, bicycle) number to spawn
    * pedestrian number : Pedestrian number to spawn
    * iteration : Number of trial

If you want to collect data more than 30 seconds, change `save_carla_data.py`
```python
# L181
if data_length > 30: # <- Now 30 seconds, change here to any value
    if travel_dist < 20:
        print('travel distance is less than 20m in 30sec -> exit')
        exit()
    else:
        return
```
### Run
```bash
# run carla
./path/to/CARLA/CarlaUE4.sh
# start logging
bash save_carla_data.py
```

### Output
* Waypoint step data : filename_prefix_iteration_dist.pickle
* Time step data : filename_prefix_iteration_time.pickle
* Carla log : filename_prefix_iteration.log

## Simulate with ROS
Play logged data with ROS

* Play Waypoint step data.
* When the ego_vehicle steps on a waypoint, publish agent information based on the log data at the waypoint
* Publish agents obstacle data which can be recognized by Autoware

### Requirement
* Autoware.ai (1.13)
* ROS (melodic)

### Setup
Configure `auto_logging_ros.sh`
* file_list : Directory which contains Waypoint step data (_dist.pickle)
* out_path : Directory to save log data

### Run
```bash
# Start autoware simulation (wf_simulator)
source /path/to/autoware/install/setup.bash
roslaunch autoware.launch
# start logging
bash auto_logging_ros.sh
```

### Output
* Time step data :
