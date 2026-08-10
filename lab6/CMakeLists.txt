cmake_minimum_required(VERSION 3.16)
project(lab_6)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(glog REQUIRED)
find_package(gflags REQUIRED)
find_package(lab_5 REQUIRED)
find_package(opengv REQUIRED)
find_package(OpenCV 4.0.0 REQUIRED)

add_executable(pose_estimation src/pose_estimation.cpp)
ament_target_dependencies(pose_estimation
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  cv_bridge
  image_transport
  message_filters
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  lab_5)
target_link_libraries(pose_estimation
  lab_5::feature_trackers
  glog::glog
  gflags::gflags
  opengv)
target_include_directories(pose_estimation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS pose_estimation DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/bags")
  install(DIRECTORY bags DESTINATION share/${PROJECT_NAME})
endif()

ament_package()
