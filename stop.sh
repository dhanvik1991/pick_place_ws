#!/usr/bin/env bash
# Stop a running simulation inside the container (the container itself keeps running).
docker exec pick_place pkill -INT -f "ros2 launch" 2>/dev/null
sleep 3
docker exec pick_place pkill -9 -f "ign gazebo|ruby|gz_ros2|move_group|rviz2|parameter_bridge|robot_state_publisher|static_transform|controller_manager|box_spawner|box_detector|pick_place.py" 2>/dev/null
exit 0
