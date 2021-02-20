cmake_minimum_required(VERSION 2.8.3)
project(fog)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge sensor_msgs std_msgs nav_msgs pcl_ros tf2_ros pcl_conversions message_filters)
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  # or if you use C: ${OpenMP_C_FLAGS}
  set(OpenMP_LIBS gomp)
endif()

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(../libraries/include)
include_directories(${OpenMP_INCLUDE_DIRS})

catkin_package(CATKIN_DEPENDS DEPENDS roscpp cv_bridge sensor_msgs std_msgs nav_msgs pcl_ros tf2_ros pcl_conversions message_filters)

add_executable(acc_space_node src/acc_space2.cpp ../libraries/src/processcloud.cpp ../libraries/src/dynamixelservos.cpp)
target_compile_options(acc_space_node PRIVATE ${OpenMP_FLAGS})
add_dependencies(acc_space_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(acc_space_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${OpenMP_LIBS})
