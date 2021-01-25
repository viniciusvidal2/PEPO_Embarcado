cmake_minimum_required(VERSION 2.8.3)
project(communication)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp message_generation cv_bridge sensor_msgs std_msgs image_transport pcl_ros image_geometry tf actionlib_msgs message_filters pluginlib rosconsole roslib compressed_image_transport)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  # or if you use C: ${OpenMP_C_FLAGS}
  set(OpenMP_LIBS gomp)
endif()

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenMP_INCLUDE_DIRS})

add_service_files(
  FILES
  state.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  DEPENDS roscpp cv_bridge message_generation sensor_msgs std_msgs image_transport pcl_ros image_geometry
)

add_executable(encaminha_dados_desktop src/encaminha_dados_desktop.cpp ../libraries/src/processcloud.cpp)
target_compile_options(encaminha_dados_desktop PRIVATE ${OpenMP_FLAGS})
add_dependencies(encaminha_dados_desktop ${catkin_EXPORTED_TARGETS})
target_link_libraries(encaminha_dados_desktop ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${OpenMP_LIBS})

add_executable(system_state_node src/system_state.cpp)
add_dependencies(system_state_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(system_state_node ${catkin_LIBRARIES})
