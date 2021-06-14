cmake_minimum_required(VERSION 3.1.0)
add_library(STATE SHARED
    run.cpp
    stop.cpp
    tracking.cpp
    )
target_include_directories(STATE
    PUBLIC ${PROJECT_SOURCE_DIR}/include/state/
    )
