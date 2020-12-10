#include "../include/pwmController.hpp"

using namespace PWM;
void PwmController::setDutyCycle(double cycle){}
void PwmController::setScale(double scale) {}

Steer::Steer() {
    std::cout << "PWM Steer PIN: " << PWM_PIN << std::endl;
    GPIO::setmode(GPIO::BOARD);
    GPIO::setup(PWM_PIN, GPIO::OUT, GPIO::LOW);
    pwm = new GPIO::PWM(PWM_PIN, 50);
    pwm->start(PWM_PIN);
    setScale(0);
}

void Steer::setScale(double scale){
    // scale (-1 -> 1) to duty (9 -> 5)
    if (scale > 1) { scale = 1; }
    else if (scale < -1) { scale = -1; }
    double duty = -scale * 2 + 7;
    setDutyCycle(duty);
}
void Steer::setDutyCycle(double cycle){
    std::cout << "PMW Steer PWM: " << cycle << std::endl;
    pwm->ChangeDutyCycle(cycle);
}

Throttle::Throttle() {
    std::cout << "PWM Throttle PIN: " << PWM_PIN << std::endl;
    GPIO::setmode(GPIO::BOARD);
    GPIO::setup(PWM_PIN, GPIO::OUT, GPIO::LOW);
    pwm = new GPIO::PWM(PWM_PIN, 50);
    pwm->start(PWM_PIN);
    setDutyCycle(0);
}

void Throttle::setScale(double scale){}
void Throttle::setDutyCycle(double cycle){
    std::cout << "PMW Throttle PWM: " << cycle << std::endl;
    pwm->ChangeDutyCycle(cycle);
}
