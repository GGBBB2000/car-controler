#include "../../include/state/state.hpp"

Stop::Stop() {
}

void Stop::doAction() {
}

std::string Stop::getName() {
    return "Stop";
}

State Stop::getNextState() {
    return State::STOP;
}
