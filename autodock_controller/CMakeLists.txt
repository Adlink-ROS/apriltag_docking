cmake_minimum_required(VERSION 3.5)
project(apriltag_docking)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2  REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(autodock_msgs REQUIRED)


include_directories(
  include include/
)

set(CTRL_EXEC_NAME "controller")
set(CLIENT_EXEC_NAME "docking_client")
add_executable( ${CTRL_EXEC_NAME} src/controller.cpp src/service_state.cpp)
add_executable( ${CLIENT_EXEC_NAME} src/service_client_node.cpp)

set(DEPENDENCIES
  "rclcpp"           
  "std_msgs"         
  "geometry_msgs" 
  "tf2_geometry_msgs"
  "tf2"
  "tf2_ros"
  "sensor_msgs"
  "autodock_msgs"
)

ament_target_dependencies(${CTRL_EXEC_NAME} ${DEPENDENCIES})
ament_target_dependencies(${CLIENT_EXEC_NAME} ${DEPENDENCIES})

install(TARGETS ${CTRL_EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${CLIENT_EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME})
  
install(
  DIRECTORY launch param rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
