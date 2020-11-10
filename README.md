# car_gazebo
GazeboSim Models such as a ackermann car 1:10
## Installation Requirements
* car_gazebo depends on car_msgs which are located in the pkg tuw_msgs (https://github.com/mx-car/car_msgs)
``` git clone https://github.com/mx-car/car_msgs.git YOUR_CATKIN_SRC_NEXT_TO_TUW_GAZEBO/ ```
* ros general
``` sudo apt-get install ros-noetic-robot-state-publisher ros-noetic-joint-state-publisher ros-noetic-xacro```
## Optional
Install gazebo_ros_pgks with your suitable branch
``` 
git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git 
cd gazebo_ros_pkgs
git checkout -b noetic-devel origin/noetic-devel
```
