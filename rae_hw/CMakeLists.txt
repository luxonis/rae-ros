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
find_package(sensor_msgs REQUIRED)
find_library(GPIOD_LIBRARY NAMES libgpiodcxx.so)
if(NOT GPIOD_LIBRARY)
  message(FATAL_ERROR "gpiod library not found. Install apt install libgpiod-dev")
endif()

# Find mpg123 package
find_package(PkgConfig REQUIRED)
pkg_check_modules(MPG123 REQUIRED libmpg123)

# Find SNDFILE library
find_path(SNDFILE_INCLUDE_DIR sndfile.h)
find_library(SNDFILE_LIBRARY NAMES sndfile)

include_directories(include ${GST_INCLUDE_DIRS} ${MPG123_INCLUDE_DIRS} ${SNDFILE_INCLUDE_DIR} ${sensor_msgs_INCLUDE_DIRS})

ament_auto_add_library(
  ${PROJECT_NAME}
  SHARED
  src/rae_hw.cpp
  src/rae_motors.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES}  nav_msgs ALSA std_srvs)
set(dependencies
    sensor_msgs)

target_link_libraries(
  ${PROJECT_NAME}
  ${MPG123_LIBRARIES}
  ${GPIOD_LIBRARY}
  ${SNDFILE_LIBRARY}
)

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface rae_hw.xml)

install(
  TARGETS
  ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

add_executable(mic_node src/peripherals/mic.cpp)
ament_target_dependencies(mic_node rclcpp audio_msgs)
target_link_libraries(mic_node ${PROJECT_NAME})

add_executable(speakers_node src/peripherals/speakers.cpp)
ament_target_dependencies(speakers_node rclcpp audio_msgs)
target_link_libraries(speakers_node ${PROJECT_NAME})

add_executable(led_node src/peripherals/led.cpp)
ament_target_dependencies(led_node rclcpp rae_msgs)
target_link_libraries(led_node ${PROJECT_NAME})

add_executable(lcd_node src/peripherals/lcd.cpp)
ament_target_dependencies(lcd_node rclcpp sensor_msgs)
target_link_libraries(lcd_node ${PROJECT_NAME})

add_executable(battery_node src/peripherals/battery.cpp)
ament_target_dependencies(battery_node rclcpp sensor_msgs)
target_link_libraries(battery_node ${PROJECT_NAME})




add_executable(test_motors test/test_motors.cpp src/rae_motors.cpp)
target_link_libraries(test_motors ${GPIOD_LIBRARY})

add_executable(test_encoders test/test_encoders.cpp src/rae_motors.cpp)
target_link_libraries(test_encoders ${GPIOD_LIBRARY})

add_executable(test_max_speed test/test_max_speed.cpp src/rae_motors.cpp)
target_link_libraries(test_max_speed ${GPIOD_LIBRARY})

add_executable(test_speed test/test_speed.cpp src/rae_motors.cpp)
target_link_libraries(test_speed ${GPIOD_LIBRARY})
ament_target_dependencies(test_speed rclcpp geometry_msgs)


install(TARGETS

  test_motors test_encoders test_max_speed test_speed mic_node speakers_node led_node lcd_node battery_node

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
install(DIRECTORY launch config assets DESTINATION share/${PROJECT_NAME})
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


ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(
  PROGRAMS
  scripts/mock/mock_battery.py
  scripts/mock/mock_lcd.py
  scripts/mock/mock_leds.py
  scripts/mock/mock_mic.py
  scripts/mock/mock_speakers.py
  scripts/mock/mock_wheels.py
  scripts/lifecycle_manager.py
  scripts/sys_info_node.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()

