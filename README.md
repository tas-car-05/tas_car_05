tas_car_05
===============================================================================================================================

This repository was created by


Christoph Allig		christoph.allig@tum.de
Robin Heinzler		robin.heinzler@tum.de
Marcin Kasperek		marcin.kasperek@tum.de
Christopher Weber	chritopher.weber@tum.de


during the laboratory acording to the lecture Technok autonomer Systeme at Technische Universität München. 

===============================================================================================================================
Package: tas_autonomous_control
Control the velocity of the car dependent on the delivered velocity from the local planer. New implementation compared to the origin node.
-------------------------------------------------------------------------------------------------------------------------------	
forward: <----- ?????????
Detection whether the car is located on a straight track or in the curve.
If the car is located on the straight track, the velocity is higher than in a curve.
The acceleration is limited to provide a soft start.
-------------------------------------------------------------------------------------------------------------------------------	
Package: ira_laser_tools (developed at Universita di Milano)
The ira_laser_tool package is used to merge the laserscan data from the front and back laser into one scan topic. Due the fact that hector_mapping, move_base and AMCL subscribe only to one scan topic, the merging of the laser data is required. For any further information, please look into the ira_laser_tool documentation. (https://github.com/iralabdisco/ira_laser_tools)
-------------------------------------------------------------------------------------------------------------------------------	
Parameter: base_local_planner_params
changed min_val_x from 0.1 to 0.01
-------------------------------------------------------------------------------------------------------------------------------	
Directory: TAS

Adjusted launch files:

in hardware.launch

Laser		| Topic		| frame_id
---------------------------------------------------------------
front		| /scan_front	| /laser_front
back		| /scan_back	| /laser_back


in odom.launch

Transformation			| from		|to
---------------------------------------------------------------
virtual_laser_to_laser_front	| /laser	| /laser_front
virtual_laser_to_laser_back	| /laser	| /laser_back
base_link_to_laser		| /base_link	| /laser

-------------------------------------------------------------------------------------------------------------------------------	
move_base_task_1:
- ist identisch mit move_base
- anpassung: initial pose = Startposition 1

move_base_task_2:
- ist identisch mit move_base
- anpassung: initial pose = Startposition 2

run_rviz_task_1.launch:	Zum Starten vom Rundkurs
- ist identisch mit run_rviz.launch
- anstatt move_base, wird move_base_task_1 gestartet
  und damit die Initial Pose für den Rundkurs gesetzt

run_rviz_task_2.launch:	Zum Starten vom Slalom
- ist identisch mit run_rviz.launch
- anstatt move_base, wird move_base_task_2 gestartet
  und damit die Initial Pose für das Slalom gesetzt

WICHTIG: die main.cpp (in /simple_navigation_goals/src) 
muss entsprechend der task angepasst werden, sodass 
die Waypoints stimmen!!
-------------------------------------------------------------------------------------------------------------------------------	


