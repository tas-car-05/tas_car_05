tas_car_05
===============================================================================================================================

This repository was created by

```
Christoph Allig		christoph.allig@tum.de
Robin Heinzler		robin.heinzler@tum.de
Marcin Kasperek		marcin.kasperek@tum.de
Christopher Weber	chritopher.weber@tum.de
```

during the laboratory according to the lecture "Technik autonomer Systeme" at Technische Universität München. 

***

#### Package: tas_autonomous_control
The package tas_autonomous_control controls the velocity of the car dependent on the delivered velocity from the local planer. 

**New implementation compared to the origin node**

Detection whether the car is located on a straight track or in the curve.
If the car is located on the straight track, the velocity is higher than in a curve.
For curve detection the algorithm subscribes to the topic slam_out_pose, which contains the robots pose without covariance.
The acceleration is limited to provide a soft start.

***

#### Package: simple_navigation_goals
The simple_navigation_package sends fixed goals, which are stored in a queue.
The goals were recorded for the first and second task.

The "start_task.sh" file in the root directory is a shell script which starts the simple_navigation_goals_node.

***

#### Package: ira_laser_tools (developed at Universita di Milano)
The ira_laser_tool package is used to merge the laserscan data from the front and back laser into one scan topic. Due the fact that hector_mapping, move_base and AMCL subscribe only to one scan topic, the merging of the laser data is required. For any further information, please look into the ira_laser_tool documentation. (https://github.com/iralabdisco/ira_laser_tools)

***

#### Parameter: base_local_planner_params
changed min_val_x from 0.1 to 0.01

***

#### Directory: TAS
Adjusted launch files.

in **hardware.launch**

|Laser		| Topic		| frame_id|
|----------------|---------------|--------------|
|front		| /scan_front	| /laser_front|
|back		| /scan_back	| /laser_back|


in **odom.launch**

|Transformation			| from		|to|
|--------------------------------|---------------|--------------|
|virtual_laser_to_laser_front	| /laser	| /laser_front|
|virtual_laser_to_laser_back	| /laser	| /laser_back|
|base_link_to_laser		| /base_link	| /laser|

***


#### Launch files and necessary changes to start task 1 and 2
	
######move_base_task_1:
- identical to move_base
- changes: initial pose = start position 1

######move_base_task_2:
- identical to move_base
- changes: initial pose = start position 2

######run_rviz_task_1.launch:	for launching task 1 - circuit
- identical to run_rviz.launch
- launching move_base_task_2 instead of move_base

######run_rviz_task_2.launch:	for launching task 2 - slalom 
- identical to run_rviz.launch
- launching move_base_task_2 instead of move_base

**IMPORTANT**: In the main.cpp (in /simple_navigation_goals/src) you have to commend the waypoint-setting of the other task out!

***

Usefull commands
-------------------------------------------------------------------------------------------------------------------------------	

Executes several launch-files for hardware, odom, move_base and rviz:
```
roslaunch ~/catkin_ws/src/tas_car_05/tas/launch/run_rviz.launch
```

Shell-script that executes simple_navigation_goals_node:
```
~/catkin_ws/src/tas_car_05/start_task
```

Opens graphical table of relationships between nodes/topics:
```
rosrun rqt_graph rqt_graph
```

Creates a pdf-file with the relationships of the existing frames and their transformations:
```
rosrun tf view_frames
```


