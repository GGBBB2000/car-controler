#ifndef INCLUDE_PWM_CONTROLLER
#define INCLUDE_PWM_CONTROLLER
#include <JetsonGPIO.h>
#include <memory>
#include <iostream>
#endif // INCLUDE_PWM_CONTROLER

namespace PWM {
    class PwmController {
        public:
            virtual void setScale(double scale);

        private:
            virtual void setDutyCycle(double freq);
    };

    class Steer : public PwmController {
        public:
            Steer();
            void setScale(double scale);
            void setDutyCycle(double freq);

        private:
            const int PWM_PIN = 33;
            GPIO::PWM *pwm;
    };

    class Throttle : public PwmController {
        public:
            Throttle();
            void setScale(double scale);
            void setDutyCycle(double freq);

        private:
            const int ENABLE_PIN = 0; //TODO
            const int PWM_PIN = 32;
            GPIO::PWM *pwm;
    };
}
