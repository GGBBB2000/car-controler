#include "../include/pwmController.hpp"
#include <memory>
#include <iostream>
#include <chrono>
#include <thread>
#include <signal.h>
#include <unistd.h>

void steerTest() {
    std::cout << "SteerTest:begin..." << std::endl;
    PWM::Steer *steer = new PWM::Steer();
    for (int i = 0; i < 4; i++) {
        double dutyCycle = i + 6;
        steer->setDutyCycle(dutyCycle);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    steer->setDutyCycle(7.5);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "SteerTest:end" << std::endl;
}

void steerScalingTest() {
    std::cout << "SteerTest:begin..." << std::endl;
    PWM::Steer *steer = new PWM::Steer();
    for (int i = 0; i < 11; i++) {
        double dutyCycle = i / 10.0;
        steer->setScale(dutyCycle);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    for (int i = 0; i < 11; i++) {
        double dutyCycle = -i / 10.0;
        steer->setScale(dutyCycle);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    steer->setScale(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "SteerTest:end" << std::endl;
}

void throttleTest() {
    std::cout << "ThrottleTest:begin..." << std::endl;
    PWM::Throttle *throttle = new PWM::Throttle();
    for (int i = 0; i < 30; i++) {
        double dutyCycle = i;
        throttle->setDutyCycle(dutyCycle);
        std::cout << "duty cycle: " << dutyCycle << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << "ThrottleTest:end" << std::endl;
}

int main() {
    std::cout << "Test begin..." << std::endl;
    //steerTest();
    steerScalingTest();
    //throttleTest();
    std::cout << "Test end" << std::endl;
}

