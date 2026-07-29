cmake_minimum_required(VERSION 3.16)
project(lab_5 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# Note: clang-19 is used via lab5_container_env. C++20 is required because
# Ubuntu OpenCV 4.5 headers do not compile as C++23/26.

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(glog REQUIRED)
find_package(OpenCV 4.0.0 REQUIRED)

# Global include so CLion/compile_commands.json always see project headers.
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(feature_trackers src/feature_tracker.cpp
                             src/sift_feature_tracker.cpp
                             src/akaze_feature_tracker.cpp
                             src/orb_feature_tracker.cpp
                             src/brisk_feature_tracker.cpp
                             src/lk_feature_tracker.cpp)

ament_target_dependencies(feature_trackers OpenCV)
target_link_libraries(feature_trackers glog::glog)
target_include_directories(feature_trackers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(track_features src/track_features.cpp)
ament_target_dependencies(track_features
  rclcpp OpenCV cv_bridge image_transport sensor_msgs)
target_link_libraries(track_features feature_trackers glog::glog)
target_include_directories(track_features PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(self_flow src/self_flow.cpp)
ament_target_dependencies(self_flow
  rclcpp OpenCV cv_bridge image_transport sensor_msgs)
target_link_libraries(self_flow feature_trackers glog::glog)
target_include_directories(self_flow PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS feature_trackers
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(TARGETS track_features self_flow DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/images")
  install(DIRECTORY images DESTINATION share/${PROJECT_NAME})
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/bags")
  install(DIRECTORY bags DESTINATION share/${PROJECT_NAME})
endif()

ament_package()
