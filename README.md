## Moveit2 planner for collision avoidance
Launch Moveit config with collision and master node
```
ros2 launch moveit_planner moveit_planner.launch.py use_fake_hardware:=true # or false with real robot
```
This launch file automatically launch:
 - ros2 ur drivers
 - ros2 moveit config
 - ros2 rviz2 with moveit config
 - ros2 collision spawner

To move the robot in random pose run the following command:
```
ros2 run moveit_planner moveit_commander_node
```