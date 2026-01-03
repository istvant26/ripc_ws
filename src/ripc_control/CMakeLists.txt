cmake_minimum_required(VERSION 3.8)
project(ripc_control)
find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_package()

install(PROGRAMS
  scripts/send_thrust.py
  DESTINATION lib/${PROJECT_NAME}
)