cd $WS01_DIR
catkin_make clean
catkin_make
source devel/setup.bash
gnome-terminal -e "roslaunch car_gazebo_models empty_world.launch"
sleep 10
gnome-terminal -e "rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=r0/cmd_vel"
sleep 1
roslaunch car_gazebo_models car.gazebo.launch
