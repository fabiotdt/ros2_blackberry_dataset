#!/bin/bash

# Launch the RealSense and berry saver nodes
ros2 launch launch_package launch_pipeline.py run_realsense:=true run_saver:=true &LAUNCH_PID=$!

# Wait a little to make sure nodes are up
sleep 2

# Now run ur_publisher with terminal access
ros2 run ur_simulator ur_publisher

# When ur_publisher exits, kill the launch file process
kill -TERM -"$LAUNCH_PGID"