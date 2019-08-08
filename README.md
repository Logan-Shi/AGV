# AGV
ackermann autonomous driving

## Depency

```
sudo apt-get install ros-kinetic-ros-controllers
sudo apt-get install ros-kinetic-gazebo-ros-control
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
can basically move by himseld but idk where exactly he's going
