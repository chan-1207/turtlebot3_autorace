cmake_minimum_required(VERSION 3.5)
project(turtlebot3_autorace_construction)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

install(PROGRAMS
  nodes/tracking_construction.py
  nodes/tracked_construction_visualization.py
  DESTINATION lib/${PROJECT_NAME}    # 설치 경로
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
