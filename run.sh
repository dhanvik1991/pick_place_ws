#!/usr/bin/env bash
# Start the container (building the image the first time) and launch the full cell.
#   ./run.sh               Gazebo GUI + automatic pick-and-place
#   ./run.sh rviz:=true    also open RViz (planning scene, trajectories, camera debug view)
#   ./run.sh auto:=false   bring up the cell without the pick-and-place node
set -e
cd "$(dirname "$0")"
# allow GUI apps from the container (same UID) to use the X server; local user only
xhost +SI:localuser:"$(id -un)" > /dev/null
docker compose up -d --build
docker exec pick_place bash -c 'source /opt/ros/humble/setup.bash && cd /ws && colcon build --symlink-install 2>&1 | tail -2'
exec docker exec -it pick_place bash -c "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && ros2 launch pick_place_sim factory.launch.py $*"
