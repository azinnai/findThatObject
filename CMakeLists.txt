set (CMAKE_CXX_STANDARD 11)


cmake_minimum_required(VERSION 2.6 FATAL_ERROR)


set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE}")
message("CMAKE_CXX_FLAGS_RELEASE is ${CMAKE_CXX_FLAGS}")

project(findThatObject)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

file(GLOB SOURCES "src/*.cpp")

add_executable(findThatObject ${SOURCES})
set (CMAKE_BUILD_TYPE Release)
target_link_libraries (findThatObject ${PCL_LIBRARIES})
