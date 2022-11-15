Package Creation
	- Change to course directory
		○ cmd:
			§ cd /home/ros/catkin_ws/src/course_dir
	- Create the package called "a2"
		○ cmd:
			§ catkin_create_pkg a2 rospy turtlesim geometry_msgs
	- Make "scripts" folder in pkg "a2"
		○ cmd:
			§ Mkdir ./a2/scripts
	- Place ros files in "scripts" folder
		○ i.e. 'target_publisher.py', 'pose_reporter.py' and 'robot_driver.py'
	- Change to 'scripts' directory
		○ cmd:
			§ Cd ./a2/scripts
	- Make files executable
		○ cmd:
			§ chmod +x target_publisher.py pose_reporter.py robot_driver.py
	- Build package
		○ comd:
			§ catkin build a2
	- Source pkg
		○ cmd:
			§ source ~/catkin_ws/devel/setup.bash

Node Sequence
	- Term 1
		○ cmd:
			§ roscore
	- Order does not matter past this point but here is a possible order
	- Term 2
		○ cmd:
			§ rosrun turtlesim turtlesim_node 
	- Term 3
		○ cmd:
			§ rosrun a2 robot_driver.py
	- Term 4 
		○ cmd:
			§ rosrun a2 pose_reporter.py
	- Term 5
		○ cmd:
			§ rosrun a2 target_publisher.py
		○ Enter values for x and y
		○ If you want to input new coords for x and y
			§ Cntrl + C
			§ rosrun a2 target_publisher.py