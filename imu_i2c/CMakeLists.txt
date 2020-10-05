cmake_minimum_required(VERSION 2.8.3)
project(imu_i2c)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp rospy)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(CATKIN_DEPENDS DEPENDS roscpp rospy)

