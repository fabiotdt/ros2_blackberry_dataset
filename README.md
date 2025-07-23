## Moveit2 planner for collision avoidance
Launch Moveit config with collision and master node
```
ros2 launch moveit_planner moveit_planner.launch.py
```
The call the service `/move_to_pose` passing a `PoseStamped` to it to move the robot arm towards a specific pose. This pose will be the next goal for the robot arm to reach.
