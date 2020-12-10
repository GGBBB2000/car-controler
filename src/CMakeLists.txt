cmake_minimum_required(VERSION 3.1.0)
add_library(SRC SHARED
        carController.cpp
        pwmController.cpp
        measurementPoint.cpp
	)
target_include_directories(SRC
    PUBLIC ${PROJECT_SOURCE_DIR}/include
    )
