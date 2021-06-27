#include "../../include/state/state.hpp"

Stop::Stop(std::shared_ptr<StreamManager> st) {
    this->streamManager = st;
}

void Stop::doAction() {
}

std::string Stop::getName() {
    return "Stop";
}

State Stop::getNextState() {
    return State::STOP;
}
