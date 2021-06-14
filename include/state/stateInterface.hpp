#ifndef INCLUDE_STATE_INTERFACE
#define INCLUDE_STATE_INTERFACE
#include <string>

template <typename StateType>
class StateInterface {
    public:
    StateInterface() = default;
    virtual void doAction() = 0; // 状態
    virtual std::string getName() = 0;
    virtual StateType getNextState() = 0;
};
#endif
