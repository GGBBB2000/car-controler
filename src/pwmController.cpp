#include "../include/pwmController.hpp"

Steer::Steer() {
   setScale(0);
}

void Steer::setScale(double scale) {
    try {
        this->sendRequest(scale).wait();
    } catch (const std::exception& e) {
        std::cout << "Steer send request failed" << std::endl;
    }
}

pplx::task<void> Steer::sendRequest(double scale) {
    return pplx::create_task([s = scale] {
                std::string url = "http://localhost:1065/steer/";
                url += std::to_string(s);
                url += "/";
                http_client client(url);
                return client.request(methods::GET);
            })
    .then([](http_response response){
            if (response.status_code() == status_codes::OK) {
            }
            });
}

Throttle::Throttle() {
}

void Throttle::setScale(double scale) {
    try {
        this->sendRequest(scale).wait();
    } catch (const std::exception& e) {
        std::cout << "exception" << std::endl;
    }
}

pplx::task<void> Throttle::sendRequest(double scale) {
    return pplx::create_task([s = scale] {
                std::string url = "http://localhost:1065/throttle/";
                url += std::to_string(s);
                url += "/";
                std::cout << url << std::endl;
                http_client client(url);
                return client.request(methods::GET);
            })
    .then([](http_response response){
            if (response.status_code() == status_codes::OK) {
            }
            });
}
