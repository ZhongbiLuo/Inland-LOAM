cmake_minimum_required(VERSION 3.5)
project(inland_loam)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cloud_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 3.3.7 REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED QUIET)
find_package(Boost REQUIRED COMPONENTS thread serialization timer)
# Find OctoMap dependencies
find_package(octomap REQUIRED)
find_package(octomap_ros REQUIRED)
find_package(octomap_msgs REQUIRED)



message(STATUS "${LIB_NAME} Ros version : $ENV{ROS_DISTRO}")
if ($ENV{ROS_DISTRO} MATCHES "iron")
  add_definitions(-DUSE_IRON_VERSION)
elseif ($ENV{ROS_DISTRO} MATCHES "humble")
  add_definitions(-DUSE_HUMBLE_VERSION)
elseif ($ENV{ROS_DISTRO} MATCHES "galactic")
  add_definitions(-DUSE_GALACTIC_VERSION)
elseif ($ENV{ROS_DISTRO} MATCHES "foxy")
  add_definitions(-DUSE_FOXY_VERSION)
elseif ($ENV{ROS_DISTRO} MATCHES "eloquent")
  add_definitions(-DUSE_ELOQUENT_VERSION)
elseif ($ENV{ROS_DISTRO} MATCHES "dashing")
  add_definitions(-DUSE_DASHING_VERSION)
elseif ($ENV{ROS_DISTRO} MATCHES "noetic")
  add_definitions(-DUSE_ROS_NORTIC_VERSION)
elseif ($ENV{ROS_DISTRO} MATCHES "melodic")
  add_definitions(-DUSE_ROS_MELODIC_VERSION)
endif()

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})
ament_export_include_directories(include)
include_directories(${OCTOMAP_INCLUDE_DIRS})


set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  cloud_msgs
)

add_executable(${PROJECT_NAME} src/imageProjection.cpp src/featureAssociation.cpp src/mapOptmization.cpp src/transformFusion.cpp src/main.cpp)
target_link_libraries(${PROJECT_NAME} 
  ${PCL_LIBRARIES} gtsam 
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies} Eigen3 octomap_msgs)
target_link_libraries(${PROJECT_NAME} ${OCTOMAP_LIBRARIES})

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
