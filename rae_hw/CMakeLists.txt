cmake_minimum_required(VERSION 3.8)
project(rae_hw)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(BUILD_SHARED_LIBS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_library(GPIOD_LIBRARY NAMES libgpiodcxx.so)
if(NOT GPIOD_LIBRARY)
  message(FATAL_ERROR "gpiod library not found. Install apt install libgpiod-dev")
endif()

include_directories(include)

add_library(
  rae_hw
  SHARED
  src/rae_hw.cpp
  src/rae_motors.cpp
)
target_include_directories(
  rae_hw
  PUBLIC
  include
)
ament_target_dependencies(
  rae_hw
  hardware_interface
  rclcpp
  rclcpp_lifecycle
)
target_link_libraries(rae_hw ${GPIOD_LIBRARY})

# prevent pluginlib from using boost
target_compile_definitions(rae_hw PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface rae_hw.xml)

install(
  TARGETS
  rae_hw
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

add_executable(battery src/battery.cpp)
ament_target_dependencies(battery rclcpp sensor_msgs)

add_executable(test_motors test/test_motors.cpp src/rae_motors.cpp)
target_link_libraries(test_motors ${GPIOD_LIBRARY})

add_executable(test_encoders test/test_encoders.cpp src/rae_motors.cpp)
target_link_libraries(test_encoders ${GPIOD_LIBRARY})

add_executable(test_max_speed test/test_max_speed.cpp src/rae_motors.cpp)
target_link_libraries(test_max_speed ${GPIOD_LIBRARY})


add_executable(test_speed test/test_speed.cpp src/rae_motors.cpp)
target_link_libraries(test_speed ${GPIOD_LIBRARY})
ament_target_dependencies(test_speed rclcpp geometry_msgs)

ament_target_dependencies(battery rclcpp sensor_msgs)

install(TARGETS
  battery test_motors test_encoders test_max_speed test_speed

  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_rae_hw test/test_rae_hw.cpp)
  target_include_directories(test_rae_hw PRIVATE include)
  ament_target_dependencies(
    test_rae_hw
    hardware_interface
    pluginlib
    ros2_control_test_assets
  )
endif()
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_export_include_directories(
  include
)
ament_export_libraries(
  rae_hw
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
