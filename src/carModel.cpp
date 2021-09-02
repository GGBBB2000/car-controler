#include "../include/carModel.hpp"

CarModel::CarModel() {
    std::cout << "Using librealsense - " << RS2_API_VERSION_STR << std::endl;
    std::shared_ptr<StreamManager> stPtr = std::make_shared<StreamManager>();
    stateMap.insert(std::make_pair(State::INIT, std::make_shared<Init>(stPtr)));
    stateMap.insert(std::make_pair(State::RUNNING, std::make_shared<Run>(stPtr)));
    stateMap.insert(std::make_pair(State::STOP, std::make_shared<Stop>(stPtr)));
    stateMap.insert(std::make_pair(State::TRACKING, std::make_shared<Tracking>(stPtr)));
}

void CarModel::run() {
    std::shared_ptr<StateInterface<State>> state;
    auto nextState = State::RUNNING;//TODO あとで初期状態を作る
    while (nextState != State::STOP) {
        state =  stateMap[nextState];
        state->doAction();
        std::cout << state->getName() << std::endl;
        nextState = state->getNextState();
    }
}
