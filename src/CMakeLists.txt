cmake_minimum_required(VERSION 3.1.0)
add_subdirectory(state)
add_library(SRC SHARED
    carModel.cpp
    pwmController.cpp
    measurementPoint.cpp
    walls.cpp
    streamManager.cpp
    )
target_include_directories(SRC
    PUBLIC ${PROJECT_SOURCE_DIR}/include
    )
