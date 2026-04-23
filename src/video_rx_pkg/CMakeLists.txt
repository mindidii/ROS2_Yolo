cmake_minimum_required(VERSION 3.8)
project(video_rx_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sentinel_interfaces REQUIRED)
find_package(Boost REQUIRED)

add_executable(video_rx_node
  src/video_rx_node.cpp
  src/frame_assembler.cpp
  src/udp_receiver.cpp
)

ament_target_dependencies(video_rx_node
  rclcpp
  sensor_msgs
  cv_bridge
  sentinel_interfaces
)

target_link_libraries(video_rx_node
  ${OpenCV_LIBS}
)

target_include_directories(video_rx_node PRIVATE
  ${Boost_INCLUDE_DIRS}
)

install(TARGETS
  video_rx_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
