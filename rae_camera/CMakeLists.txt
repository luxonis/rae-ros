cmake_minimum_required(VERSION 3.8)
project(rae_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
add_compile_options(-g)

## is used, also find other catkin packages
if(POLICY CMP0057)
    cmake_policy(SET CMP0057 NEW)
endif()
cmake_policy(SET CMP0074 NEW)
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
find_package(rclcpp_components REQUIRED)
find_package(rtabmap_slam REQUIRED)
find_package(depthai_ros_driver REQUIRED)
find_package(ira_laser_tools REQUIRED)
find_package(image_proc REQUIRED)
find_package(laserscan_kinect REQUIRED)


find_package(OpenCV 4 REQUIRED COMPONENTS imgproc highgui)
set(dependencies
  camera_info_manager
  cv_bridge
  depthai_bridge
  rclcpp
  sensor_msgs
  stereo_msgs
  std_msgs
  vision_msgs
  depthai
  OpenCV
)


set(IPC_DEPS_RTABMAP
rclcpp
rtabmap_slam
depthai_ros_driver
ira_laser_tools
laserscan_kinect
image_proc
)

include_directories(
  include
  ${ament_INCLUDE_DIRS}
)

add_executable(camera src/camera.cpp)
add_executable(stereo_depth src/stereo_depth.cpp)
add_executable(yolo src/yolo.cpp)
add_executable(stereo_pair src/stereo_pair.cpp)
add_executable(perception_ipc_rtabmap src/perception_ipc_rtabmap)


ament_target_dependencies(camera
${dependencies})
install(TARGETS
camera
DESTINATION lib/${PROJECT_NAME})

ament_target_dependencies(stereo_depth
${dependencies})

install(TARGETS
stereo_depth
DESTINATION lib/${PROJECT_NAME})


ament_target_dependencies(stereo_pair
${dependencies})

install(TARGETS
stereo_pair
DESTINATION lib/${PROJECT_NAME})

ament_target_dependencies(yolo
${dependencies})

install(TARGETS
yolo
DESTINATION lib/${PROJECT_NAME})

ament_target_dependencies(perception_ipc_rtabmap ${IPC_DEPS_RTABMAP})
install(TARGETS perception_ipc_rtabmap
DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(${IPC_DEPS_RTABMAP})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_package()
