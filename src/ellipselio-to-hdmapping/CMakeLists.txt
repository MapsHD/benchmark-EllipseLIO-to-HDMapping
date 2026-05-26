cmake_minimum_required(VERSION 3.8)
project(ellipselio-to-hdmapping)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS common)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# LASzip library
add_subdirectory(src/3rdparty/LASzip)
include_directories(src/3rdparty/LASzip/include)
include_directories(src/3rdparty/LASzip/include/laszip)
include_directories(src/3rdparty/LASzip/dll)

include_directories(
  ${PCL_INCLUDE_DIRS}
)

add_executable(listener
  src/ellipselio_odometry_subscriber.cpp
  src/laz_writer.cpp
)

ament_target_dependencies(listener
  rclcpp
  sensor_msgs
  nav_msgs
  rosbag2_cpp
)

target_link_libraries(listener
  Eigen3::Eigen
  laszip
  ${PCL_LIBRARIES}
)

install(TARGETS listener
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
