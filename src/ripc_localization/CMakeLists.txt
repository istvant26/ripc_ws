cmake_minimum_required(VERSION 3.8)
project(ripc_localization)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
# Install the nodes directory (Python scripts)
# Install the bridge script so ROS 2 can find it
install(PROGRAMS
  nodes/rtcm_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
