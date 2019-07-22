# AGV
ackermann autonomous driving

## Depency

```
sudo apt-get install ros-kinetic-ros-controllers
```

## Bringup

* simulator  
`roslaunch racecar_simulator simulate.launch`

* gazebo  
`roslaunch racecar_gazebo racecar.launch`

* keyboard teleop  
`rosrun racecar_control keyboard_teleop.py`
