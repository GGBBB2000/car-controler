#include "../../include/state/state.hpp"

Init::Init(std::shared_ptr<StreamManager> st) {
}


void Init::doAction() {
}

std::string Init::getName() {
    return "INIT";
}

State Init::getNextState() {
    return State::INIT;
}
