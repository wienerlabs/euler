cmake_minimum_required(VERSION 3.10)
project(euler_gazebo_plugin)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(gazebo REQUIRED)

include_directories(
    ${GAZEBO_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/../../../include
)

link_directories(${GAZEBO_LIBRARY_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(euler_gazebo_plugin SHARED
    euler_gazebo_plugin.cpp
)

target_link_libraries(euler_gazebo_plugin
    ${GAZEBO_LIBRARIES}
    ${CMAKE_SOURCE_DIR}/../../../build/libeuler.a
    pthread
    m
)

target_compile_definitions(euler_gazebo_plugin PRIVATE EULER_SIMULATION=1)

install(TARGETS euler_gazebo_plugin
    LIBRARY DESTINATION lib
)

