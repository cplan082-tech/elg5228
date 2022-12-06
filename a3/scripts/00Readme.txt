- Navigate to "course_dir"
	- cmd: cd ~/catkin_ws/src/course_dir/

- Create pkg a3
	- cmd: catkin_create_pkg a3 rospy sensor_msgs geometry_msgs nav_msgs tf

- Make "scripts" folder in pkg "a3"
	- cmd:
		- mkdir ./a3/scripts

- Place ros files in "scripts" folder
	- i.e. 'sensed_object.py' and 'navigate_robot.py'

-build pkg
	- cmd:
		- catkin build a3


- Source pkg
	- cmd:
		- source ~/catkin_ws/devel/setup.bash

- enable LiDAR
	- cmd:
		- export HUSKY_LMS1XX_ENABLED=1

- launch husky in gazebo
	- cmd: roslaunch husky_gazebo husky_empty_world.launch

- Terminal 2:
	- run 'sensed_object.py' node
		- cmd: rosrun a3 sensed_object.py
- Term 3:
	- run 'navigate_robot.py' node
		-cmd: rosrun a3 navigate_robot.py



