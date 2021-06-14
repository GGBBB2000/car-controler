#ifndef INCLUDE_PWM_CONTROLLER
#define INCLUDE_PWM_CONTROLLER
#include <memory>
#include <iostream>
#include <cpprest/http_client.h>
#include <string>
#endif // INCLUDE_PWM_CONTROLER

using namespace utility;
using namespace web;
using namespace web::http;
using namespace web::http::client;
using namespace concurrency::streams;

struct PwmController {
    virtual void setScale(double scale) = 0;
    virtual pplx::task<void> sendRequest(double scale) = 0;
};

struct Steer : public PwmController {
    public:
        Steer();
        void setScale(double scale) override;
        pplx::task<void> sendRequest(double scale) override;
};

struct Throttle : public PwmController {
    public:
        Throttle();
        void setScale(double scale) override;
        pplx::task<void> sendRequest(double scale) override;
};
