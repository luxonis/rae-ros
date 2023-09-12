cmake_minimum_required(VERSION 3.8)
project(rae_hw)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(BUILD_SHARED_LIBS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(ALSA REQUIRED)

find_library(GPIOD_LIBRARY NAMES libgpiodcxx.so)
if(NOT GPIOD_LIBRARY)
  message(FATAL_ERROR "gpiod library not found. Install apt install libgpiod-dev")
endif()

include_directories(include)

ament_auto_add_library(
  ${PROJECT_NAME}
  SHARED
  src/rae_hw.cpp
  src/rae_motors.cpp
  src/peripherals/battery.cpp
  src/peripherals/lcd.cpp
  src/peripherals/led.cpp
  src/peripherals/mic.cpp
  src/peripherals/speakers.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES} ALSA)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  include
)
target_link_libraries(${PROJECT_NAME} ${GPIOD_LIBRARY})

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

rclcpp_components_register_nodes(${PROJECT_NAME} "${PROJECT_NAME}::BatteryNode")
rclcpp_components_register_nodes(${PROJECT_NAME} "${PROJECT_NAME}::LCDNode")
rclcpp_components_register_nodes(${PROJECT_NAME} "${PROJECT_NAME}::LEDNode")
rclcpp_components_register_nodes(${PROJECT_NAME} "${PROJECT_NAME}::MicNode")
rclcpp_components_register_nodes(${PROJECT_NAME} "${PROJECT_NAME}::SpeakersNode")

pluginlib_export_plugin_description_file(
  hardware_interface rae_hw.xml)

install(
  TARGETS
  ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)


add_executable(test_motors test/test_motors.cpp src/rae_motors.cpp)
target_link_libraries(test_motors ${GPIOD_LIBRARY})

add_executable(test_encoders test/test_encoders.cpp src/rae_motors.cpp)
target_link_libraries(test_encoders ${GPIOD_LIBRARY})

add_executable(test_max_speed test/test_max_speed.cpp src/rae_motors.cpp)
target_link_libraries(test_max_speed ${GPIOD_LIBRARY})

add_executable(test_speed test/test_speed.cpp src/rae_motors.cpp)
target_link_libraries(test_speed ${GPIOD_LIBRARY})
ament_target_dependencies(test_speed rclcpp geometry_msgs)

# ament_target_dependencies(battery rclcpp sensor_msgs)

install(TARGETS

  test_motors test_encoders test_max_speed test_speed

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

  # ament_add_gmock(test_rae_hw test/test_rae_hw.cpp)
  # target_include_directories(test_rae_hw PRIVATE include)
  ament_target_dependencies(
    test_motors
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
  ${PROJECT_NAME}
)
ament_export_dependencies(
  ${DEPENDENCIES}
)
install(
    DIRECTORY include/
    DESTINATION include
)

ament_package()

