#ifndef INCLUDE_CAR_MODEL
#define INCLUDE_CAR_MODEL
#include <iostream>
#include "state/state.hpp"

class CarModel {
    std::map<State, std::shared_ptr<StateInterface<State>>> stateMap;

    public:
        CarModel();
        void run();
};

#endif // INCLUDE_CAR_MODEL
