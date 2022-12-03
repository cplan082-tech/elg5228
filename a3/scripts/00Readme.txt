- Navigate to "course_dir"
	- cmd: cd ~/catkin_ws/src/course_dir/

- Create pkg a3
	- cmd: catkin_create_pkg a3 rospy sensor_msg

- Make "scripts" folder in pkg "a3"
	- cmd:
		- mkdir ./a3/scripts

- Place ros files in "scripts" folder
	- i.e. 'sensed_object.py' and 'navigate_robot.py'


- Source pkg
	- cmd:
		- source ~/catkin_ws/devel/setup.bash

- enable LiDAR
	- cmd:
		- export HUSKY_LMS1XX_ENABLED=1



