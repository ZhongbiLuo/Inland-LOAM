cmake_minimum_required(VERSION 3.5)
project(mapconversion)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(mapconversion_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Find OctoMap dependencies
find_package(octomap REQUIRED)
find_package(octomap_ros REQUIRED)
find_package(octomap_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${OCTOMAP_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}  # Add the path of the rclcpp header file
)


# Create library from MapConverter class
add_library(map_conversion_class src/MapConverter.cpp)
target_include_directories(map_conversion_class PUBLIC
  ${OCTOMAP_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)
ament_target_dependencies(map_conversion_class 
  Eigen3
  rclcpp        # Add rclcpp dependency
  octomap       # Add octomap dependency
  octomap_msgs
)

# Link the library to the executable properly
add_executable(map_conversion_oct_node src/map_conversion_oct_node.cpp)
target_link_libraries(map_conversion_oct_node 
  map_conversion_class
  ${OCTOMAP_LIBRARIES}
  ${rclcpp_LIBRARIES}  # Add the rclcpp library
)  
ament_target_dependencies(map_conversion_oct_node
  visualization_msgs
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  mapconversion_msgs
  octomap_ros
  octomap
  octomap_msgs
)

# Install executables
install(TARGETS
  map_conversion_oct_node 
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
