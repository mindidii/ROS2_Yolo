cmake_minimum_required(VERSION 3.8)
project(image_preprocess_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sentinel_interfaces REQUIRED)

add_executable(image_preprocess_node
  src/image_preprocess_node.cpp
)

ament_target_dependencies(image_preprocess_node
  rclcpp
  sensor_msgs
  cv_bridge
  sentinel_interfaces
)

target_link_libraries(image_preprocess_node
  ${OpenCV_LIBS}
)

install(TARGETS
  image_preprocess_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/capture_eo_calibration_images.py
  scripts/calibrate_eo_camera.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
