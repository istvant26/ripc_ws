cmake_minimum_required(VERSION 3.8)
project(ripc_localization)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
# Install the nodes directory (Python scripts)
install(DIRECTORY nodes/
  DESTINATION share/${PROJECT_NAME}/nodes
  PATTERN "*.py"
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ
)
ament_package()
