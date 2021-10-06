cmake_minimum_required(VERSION 3.5)

set(PROJECT_NAME aruco_gate_detector)
project(${PROJECT_NAME})

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(aerostack2_core REQUIRED)
find_package(aerostack2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(cv_bridge REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


include_directories(
  include
  include/${PROJECT_NAME}
  ${realsense2_INCLUDE_DIR}
)

set(HEADER_HPP_FILES 
  include/${PROJECT_NAME}/aruco_gate_detector.hpp  
)

set(SOURCE_CPP_FILES 
  src/aruco_gate_detector.cpp
)

#add_executable(test_pose src/test_pose.cpp)
#target_link_libraries(test_pose ${realsense2_LIBRARY})
#ament_target_dependencies(test_pose rclcpp std_msgs realsense2 aerostack2_core aerostack2_msgs)

add_executable(${PROJECT_NAME}_node src/aruco_gate_detector_node.cpp ${SOURCE_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME}_node rclcpp aerostack2_core aerostack2_msgs std_msgs std_srvs sensor_msgs geometry_msgs nav_msgs cv_bridge tf2 tf2_ros tf2_geometry_msgs)


install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
