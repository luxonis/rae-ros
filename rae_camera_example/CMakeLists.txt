cmake_minimum_required(VERSION 3.8)
project(rae_camera_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(depthai CONFIG REQUIRED)
find_package(depthai_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

set(dependencies
  camera_info_manager
  cv_bridge
  depthai_bridge
  rclcpp
  sensor_msgs
  stereo_msgs
  std_msgs
  vision_msgs
)

include_directories(
  include
  ${ament_INCLUDE_DIRS}
)

  add_executable(camera src/camera.cpp)


  ament_target_dependencies(camera
                            ${dependencies})
install(TARGETS
        camera
        DESTINATION lib/${PROJECT_NAME})

ament_package()
