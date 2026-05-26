cmake_minimum_required(VERSION 3.8)
project(video_rx_pkg2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── 패키지 찾기 ───────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sentinel_interfaces REQUIRED)
find_package(Boost REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED
  gstreamer-1.0
  gstreamer-app-1.0)

# ── 실행파일 ──────────────────────────────────────────────────
add_executable(video_rx_node2
  src/video_rx_node.cpp
  src/frame_assembler.cpp
  src/udp_receiver.cpp
)

ament_target_dependencies(video_rx_node2
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  sentinel_interfaces
)

target_include_directories(video_rx_node2 PRIVATE
  ${Boost_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
)

target_link_libraries(video_rx_node2
  ${OpenCV_LIBS}
  ${GST_LIBRARIES}
)

# ── install ───────────────────────────────────────────────────
install(TARGETS
  video_rx_node2
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

# cmake_minimum_required(VERSION 3.8)
# project(video_rx_pkg2)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(cv_bridge REQUIRED)
# find_package(OpenCV REQUIRED)
# find_package(sentinel_interfaces REQUIRED)
# find_package(Boost REQUIRED)

# add_executable(video_rx_node2
#   src/video_rx_node.cpp
#   src/frame_assembler.cpp
#   src/udp_receiver.cpp
# )

# ament_target_dependencies(video_rx_node2
#   rclcpp
#   sensor_msgs
#   std_msgs
#   cv_bridge
#   sentinel_interfaces
# )

# target_link_libraries(video_rx_node2
#   ${OpenCV_LIBS}
# )

# target_include_directories(video_rx_node2 PRIVATE
#   ${Boost_INCLUDE_DIRS}
# )

# install(TARGETS
#   video_rx_node2
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(DIRECTORY
#   config
#   launch
#   DESTINATION share/${PROJECT_NAME}
# )

# ament_package()
