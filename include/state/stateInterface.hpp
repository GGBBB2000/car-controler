#ifndef INCLUDE_STATE_INTERFACE
#define INCLUDE_STATE_INTERFACE
#include <string>
#include "../streamManager.hpp"

template <typename StateType>
class StateInterface {
    public:
	//StateInterface(std::shared_ptr<StreamManager> st) = default;
	virtual void doAction() = 0; // 状態
	virtual std::string getName() = 0;
	virtual StateType getNextState() = 0;
    protected:
	std::shared_ptr<StreamManager> streamManager;	
};
#endif
