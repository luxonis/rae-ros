cmake_minimum_required(VERSION 3.22)
project(laserscan_kinect)

# Default to C99
set(CMAKE_C_STANDARD 99)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
### Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp_components REQUIRED)

set(DEPENDENCIES
rclcpp
sensor_msgs
image_geometry
image_transport
cv_bridge
rclcpp_components)

include_directories(include)

### Build
add_library(${PROJECT_NAME} SHARED
  src/laserscan_kinect.cpp
  src/laserscan_kinect_node.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})
rclcpp_components_register_nodes(${PROJECT_NAME} "${PROJECT_NAME}::LaserScanKinectNode")


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})


install(TARGETS ${PROJECT_NAME}
EXPORT ${PROJECT_NAME}Targets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

install(EXPORT ${PROJECT_NAME}Targets
      DESTINATION share/${PROJECT_NAME}/cmake)


## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
ament_export_libraries(${PROJECT_NAME})

install(
    DIRECTORY include/
    DESTINATION include
)
ament_package()