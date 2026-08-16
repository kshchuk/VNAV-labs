cmake_minimum_required(VERSION 3.16)
project(lab_8)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(glog REQUIRED)
find_package(GTSAM REQUIRED)
find_package(ultralytics_ros REQUIRED)

if(TARGET gtsam)
  set(LAB8_GTSAM_LIBS gtsam)
else()
  set(LAB8_GTSAM_LIBS ${GTSAM_LIBRARIES})
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(deliverable_1 src/deliverable_1.cpp)
ament_target_dependencies(deliverable_1
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  ultralytics_ros
  tf2
  tf2_ros)
target_link_libraries(deliverable_1 ${LAB8_GTSAM_LIBS} glog::glog)
target_include_directories(deliverable_1 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GTSAM_INCLUDE_DIR})

install(TARGETS deliverable_1 DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
