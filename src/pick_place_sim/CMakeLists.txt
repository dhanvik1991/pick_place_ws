cmake_minimum_required(VERSION 3.8)
project(pick_place_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds urdf config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/box_spawner.py
  scripts/box_detector.py
  scripts/pick_place.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/gz_utils.py DESTINATION lib/${PROJECT_NAME})

ament_package()
