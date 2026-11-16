#!/usr/bin/env bash
# Open another shell in the running container (ROS environment already sourced).
exec docker exec -it pick_place bash
