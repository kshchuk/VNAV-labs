cmake_minimum_required(VERSION 3.16)
project(lab_7)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(glog REQUIRED)
find_package(GTSAM REQUIRED)

# System GTSAM installs headers under /usr/include/gtsam and libgtsam.so.
# Prefer the imported target when available (ros-humble-gtsam / modern CMake).
if(TARGET gtsam)
  set(LAB7_GTSAM_LIBS gtsam)
else()
  set(LAB7_GTSAM_LIBS ${GTSAM_LIBRARIES})
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(deliverable_1 src/deliverable_1.cpp)
target_link_libraries(deliverable_1 ${LAB7_GTSAM_LIBS})
target_include_directories(deliverable_1 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GTSAM_INCLUDE_DIR})

add_executable(deliverable_2_3 src/deliverable_2_3.cpp)
ament_target_dependencies(deliverable_2_3
  rclcpp geometry_msgs visualization_msgs std_msgs)
target_link_libraries(deliverable_2_3 ${LAB7_GTSAM_LIBS} glog::glog)
target_include_directories(deliverable_2_3 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GTSAM_INCLUDE_DIR})

add_executable(deliverable_4 src/deliverable_4.cpp)
ament_target_dependencies(deliverable_4
  rclcpp geometry_msgs visualization_msgs std_msgs)
target_link_libraries(deliverable_4 ${LAB7_GTSAM_LIBS} glog::glog)
target_include_directories(deliverable_4 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GTSAM_INCLUDE_DIR})

add_executable(deliverable_5 src/deliverable_5.cpp)
target_link_libraries(deliverable_5 ${LAB7_GTSAM_LIBS})
target_include_directories(deliverable_5 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GTSAM_INCLUDE_DIR})

install(TARGETS deliverable_1 deliverable_2_3 deliverable_4 deliverable_5
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

ament_package()
