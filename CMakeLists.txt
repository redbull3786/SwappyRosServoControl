cmake_minimum_required(VERSION 2.8.3)
project(swappy_ros_servo_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
    roscpp
    rosconsole
    message_generation
)

## Declare ROS messages and services
add_message_files(DIRECTORY msg FILES 
    ServoState.msg
    ServoControlState.msg
)

## Generate added messages and services
generate_messages()

## Declare a catkin package
catkin_package(
    CATKIN_DEPENDS message_runtime
)

## Build node and client
include_directories(include ${catkin_INCLUDE_DIRS})

## Find all SRC Files
file (GLOB SRC_FILES
    src/protocol/*.[ch]pp
    src/configuration/*.[ch]pp
)

## Node
#add_executable(${PROJECT_NAME}_node src/node.cpp ${SRC_FILES})
#target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

## Client
add_executable(${PROJECT_NAME}_client src/client.cpp src/Wrapper.cpp ${SRC_FILES})
target_link_libraries(${PROJECT_NAME}_client ${catkin_LIBRARIES})