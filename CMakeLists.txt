set (CMAKE_CXX_STANDARD 11)


cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

#find_package(OpenMP)

#if (OPENMP_FOUND)
    #set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_DEBUG} ${OpenMP_CXX_FLAGS} ")
#endif()
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE} -fopenmp")
message("CMAKE_CXX_FLAGS_RELEASE is ${CMAKE_CXX_FLAGS}")

project(findThatObject)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

file(GLOB SOURCES "src/*.cpp")

#add_executable (pcl_visualizer_demo pcl_visualizer_demo.cpp)
add_executable(findThatObject ${SOURCES})
target_link_libraries (findThatObject ${PCL_LIBRARIES})
