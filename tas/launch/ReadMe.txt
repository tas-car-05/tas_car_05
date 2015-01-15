
-------------------------------------------------------

move_base_task_1.launch:
- ist identisch mit move_base
- anpassung: initial pose = Startposition 1

-------------------------------------------------------

move_base_task_2.launch:
- ist identisch mit move_base
- anpassung: initial pose = Startposition 2

-------------------------------------------------------

run_rviz_task_1.launch:					Zum Starten vom Rundkurs
- ist identisch mit run_rviz.launch
- anstatt move_base, wird move_base_task_1 gestartet
  und damit die Initial Pose für den Rundkurs gesetzt

-------------------------------------------------------

run_rviz_task_2.launch:					Zum Starten vom Slalom 
- ist identisch mit run_rviz.launch
- anstatt move_base, wird move_base_task_2 gestartet
  und damit die Initial Pose für das Slalom gesetzt

--------------------------------------------------------

WICHTIG: die main.cpp (in /simple_navigation_goals/src) 
muss entsprechend der task angepasst werden, sodass 
die Waypoints stimmen!!

--------------------------------------------------------