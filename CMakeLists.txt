cmake_minimum_required(VERSION 2.8.3)
project(direction_calculus)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package()

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(direction_calculus src/DirectionCalculus.cpp)

# Declare a cpp executable
 add_executable(direction_calculus_node src/direction_calculus_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(direction_calculus_node direction_calculus_generate_messages_cpp)

##Specify libraries to link a library or executable target against
 target_link_libraries(direction_calculus_node
   DirectionCalculus
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )
