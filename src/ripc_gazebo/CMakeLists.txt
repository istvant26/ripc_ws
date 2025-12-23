cmake_minimum_required(VERSION 3.8)
project(ripc_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY models
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY worlds
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

ament_package()
