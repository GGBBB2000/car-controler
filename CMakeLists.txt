cmake_minimum_required(VERSION 3.1.0)

project(car-controller)
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

include_directories(${OpenCV_INCLUDE_DIRS})
add_subdirectory(src)
add_subdirectory(test)

#add_executable(${PROJECT_NAME} src/main.cpp src/carController.cpp src/pwmController.cpp)
add_executable(${PROJECT_NAME} src/main.cpp )

target_link_libraries(${PROJECT_NAME} SRC ${realsense2_LIBRARY} ${OpenCV_LIBS} JetsonGPIO )
