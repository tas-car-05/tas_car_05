Launch files and necessary changes to start task 1 and 2
=====
move_base_task_1:
- identical to move_base
- changes: initial pose = start position 1

move_base_task_2:
- identical to move_base
- changes: initial pose = start position 2

run_rviz_task_1.launch:	for launching task 1 - circuit
- identical to run_rviz.launch
- launching move_base_task_2 instead of move_base

run_rviz_task_2.launch:	for launching task 2 - slalom 
- identical to run_rviz.launch
- launching move_base_task_2 instead of move_base

IMPORTANT: In the main.cpp (in /simple_navigation_goals/src) you have to commend the waypoint-setting of the other task out!
