cmake_minimum_required(VERSION 2.8.3)
project(pepo_obj)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge sensor_msgs std_msgs nav_msgs pcl_ros laser_geometry message_generation tf tf_conversions
)
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  # or if you use C: ${OpenMP_C_FLAGS}
  set(OpenMP_LIBS gomp)
endif()

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   comandoObj.srv
 )

generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(../libraries/include)
include_directories(${OpenMP_INCLUDE_DIRS})

catkin_package(CATKIN_DEPENDS DEPENDS roscpp cv_bridge sensor_msgs std_msgs nav_msgs pcl_ros laser_geometry message_generation)

add_executable(scanner_obj src/scanner_obj.cpp ../libraries/src/processcloud.cpp ../libraries/src/processimages.cpp)
target_compile_options(scanner_obj PRIVATE ${OpenMP_FLAGS})
add_dependencies(scanner_obj ${catkin_EXPORTED_TARGETS})
target_link_libraries(scanner_obj ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${OpenMP_LIBS})
