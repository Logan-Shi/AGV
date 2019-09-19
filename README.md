# AGV
ackermann autonomous driving

## Depency

```
sudo apt-get install ros-kinetic-ros-controllers ros-kinetic-gazebo-ros-control ros-kinetic-effort-* ros-kinetic-joint-state-*
```

## Bringup

* stage sim
`roslaunch teb_local_planner_tutorials robot_carlike_in_stage.launch`

* gazebo sim
`roslaunch racecar_gazebo racecar.launch`

* keyboard teleop  
`roslaunch base_controller teleop.launch`

* driver
`roslaunch base_controller amcl.launch`

* controller  
`roslaunch sensor_node sensor_node.launch`

* planner
`roslaunch base_controller assigner.launch`

* hilens
`rosrun base_controller hilens_display.py`

* mapping
`rosrun gmapping slam_gmapping`
`roslaunch cartographer_ros demo_rplidar.launch`

* localization and planning
`roslaunch base_controller navigation.launch`

* record bag
`roslaunch base_controller bag.launch`

## Important Directories Explained
```
.
├── base_controller
│   ├── launch
│   │   ├── amcl.launch
│   │   ├── assigner.launch------------------------bring up car and lidar
│   │   ├── bag.launch-----------------------------record bag
│   │   ├── base_controller.launch-----------------bring up car
│   │   ├── cmd_vel_mux.launch
│   │   ├── gmapping.launch
│   │   ├── laser_filter.launch
│   │   ├── move_base.launch
│   │   ├── navigation.launch----------------------localization and planning
│   │   └── teleop.launch
│   ├── param
│   │   ├── cmd_vel_mux_param.yaml
│   │   ├── costmap_converter_params.yaml
│   │   ├── laser_config.yaml
│   │   └── velocity_smoother_param.yaml
│   └── scripts
│       ├── assigner.py----------------------------planner
│       ├── avoid.py
│       ├── base_controller.py---------------------driver
│       └── hilens_display.py----------------------hilens
├── laser_filters----------------------------------laser filter node
├── racecar----------------------------------------MIT-racecar setup
├── racecar_gazebo---------------------------------gazebo simulation
├── racecar_simulator------------------------------MIT-racecar simulator
├── range_libc-------------------------------------lib for ray-tracing
├── README.md
├── rplidar_ros------------------------------------rplidar node
├── sensor_node------------------------------------controller
│   ├── launch
│   │   └── sensor_node.launch
│   └── src
│       ├── sensor_main.cpp
│       ├── sensornode.cpp
│       └── sensornode.h
├── socket-----------------------------------------hilens demo
│   ├── hilens_socket.py
│   └── tx2_socket.py
├── TA_example_labs--------------------------------some reference examples
├── teb_local_planner_tutorials--------------------teb planner configuration
│   ├── cfg
│   │   ├── amcl_params.yaml
│   │   └── carlike--------------------------------teb planner params
│   │       ├── costmap_common_params.yaml
│   │       ├── global_costmap_params.yaml
│   │       ├── local_costmap_params.yaml
│   │       └── teb_local_planner_params.yaml
│   ├── launch
│   │   └── robot_carlike_in_stage.launch----------simulation
│   ├── maps---------------------------------------map files
│   └── stage
│       ├── experiment_carlike.world---------------match world
│       ├── maze_carlike.world---------------------maze world
│       └── robots
│           ├── carlike_robot.inc------------------car
│           └── obstacle.inc-----------------------obstacle
├── teleop
└── vesc-------------------------------------------odom and cmd_vel_mux reference
```
