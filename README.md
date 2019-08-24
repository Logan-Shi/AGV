# AGV
ackermann autonomous driving

## Depency

```
sudo apt-get install ros-kinetic-ros-controllers ros-kinetic-gazebo-ros-control ros-kinetic-effort-* ros-kinetic-joint-state-*
may need CUDA support or just use my compiles lib
```

## Bringup

* simulator  
`roslaunch racecar_simulator simulate.launch`

* gazebo  
`roslaunch racecar_gazebo racecar.launch`

* keyboard teleop  
`rosrun racecar_control keyboard_teleop.py`

* controller  
`roslaunch ta_lab3 example.launch`  
Fit a line from the wall and try to keep certain distance from it. Easily run into hard corners.

`roslaunch ta_lab4 hue.launch\visual_servo.launch`  
Seems to detect a cone using vision, but won't move

`roslaunch ta_lab5 localize.launch`  
localize bot, input raw /odom, and output /pf/pose/odom

`roslaunch ta_lab5 slime.launch`  
hector_trajectory_server: Get the traveled trajectory

## Match Log

done turning with last year node, not reassuring enough but not to shabby.
try to run a lap
