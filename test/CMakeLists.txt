cmake_minimum_required(VERSION 3.1.0)
add_library(TEST STATIC
            pwmTest.cpp
	)
link_directories(/usr/local/lib/)
add_executable(pwmTest pwmTest.cpp ../src/pwmController.cpp)
target_link_libraries(pwmTest JetsonGPIO)
