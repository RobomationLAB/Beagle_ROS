cmake_minimum_required(VERSION 3.0.2)
project(camera)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
#set(CMAKE_BUILD_TYPE Release)

# set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_CXX_FLAGS "-O0 -DDEBUG -std=c++11")


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs  
  image_transport
  cv_bridge
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} 
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs
#  DEPENDS system_lib
)

find_package(OpenCV REQUIRED)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/
)

 add_executable(${PROJECT_NAME} src/main.cpp src/camera.h)
 add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
 )
