cmake_minimum_required(VERSION 3.5)
project(ros2-imu-bno055)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(core REQUIRED)

# include message types
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

#catkin_package(INCLUDE_DIRS include)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Add all the installation directories
install(DIRECTORY
  #launch
  DESTINATION share/${PROJECT_NAME}
)

add_executable(bno055_i2c_node 
  src/bno055_i2c_activity.cpp 
  src/bno055_i2c_node.cpp 
  src/watchdog.cpp
)

target_link_libraries(bno055_i2c_node ${catkin_LIBRARIES})

install(TARGETS bno055_i2c_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()