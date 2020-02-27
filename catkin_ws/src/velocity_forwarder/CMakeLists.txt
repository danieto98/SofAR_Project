cmake_minimum_required(VERSION 2.8.3)
project(velocity_forwarder)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg)

add_executable(velocity_forwarder src/velocity_forwarder.cpp)
target_link_libraries(velocity_forwarder ${catkin_LIBRARIES})
include_directories(${catkin_INCLUDE_DIRS})
add_dependencies(velocity_forwarder velocity_forwarder_velocity_forwarder_cpp)
