cmake_minimum_required(VERSION 2.8.3)
project(perception_ros)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-fopenmp")
#set(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")
add_definitions(-D_GLIBCXX_USE_C99=1)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    rosbag
    std_msgs
    sensor_msgs
    message_filters
)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(PCL REQUIRED)
if(PCL_FOUND)
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
endif ()

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp rosbag std_msgs sensor_msgs message_filters
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

############### subfolders ######################
add_subdirectory(src/lidar)

add_executable(${PROJECT_NAME}_node src/perception_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
    lidar_perception_lib
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)
