cmake_minimum_required(VERSION 3.1.01)

project(car-controller)
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)

message(STATUS "build with " ${CMAKE_CXX_COMPILER})
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_FLAGS "-fopenmp -O2 -march=native")
#set(CMAKE_CXX_FLAGS "")

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(/usr/local/cuda/lib64)
add_subdirectory(src)
#add_subdirectory(test)

add_executable(${PROJECT_NAME} src/main.cpp )

target_link_libraries(${PROJECT_NAME} SRC STATE ${realsense2_LIBRARY} ${OpenCV_LIBS} pthread
                        cpprest boost_system ssl crypto cudart)
