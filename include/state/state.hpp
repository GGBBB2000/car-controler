#ifndef INCLUDE_STATE
#define INCLUDE_STATE
#include "stateInterface.hpp"
#include <iostream>
#include <librealsense2/rs.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <tuple>
#include <ctime>
#include <chrono>
#include "../measurementPoint.hpp"
#include "../streamManager.hpp"
//#include "walls.hpp"

enum State {
    INIT,
    RUNNING, 
    TRACKING,
    RETRY,
    STOP,
};

class Init: public StateInterface<State> {
    public:
        Init(std::shared_ptr<StreamManager> st);
        void doAction();
        std::string getName();
        State getNextState();
};

class Run : public StateInterface<State> {
    public:
        Run(std::shared_ptr<StreamManager> st);
        void doAction();
        std::string getName();
        State getNextState();
};

class Stop : public StateInterface<State> {
    public:
        Stop(std::shared_ptr<StreamManager> st);
        void doAction();
        std::string getName();
        State getNextState();
};

class Tracking : public StateInterface<State> {
    public:
        Tracking(std::shared_ptr<StreamManager> st);
        void doAction();
        std::string getName();
        State getNextState();

    private:
	bool steerControll(rs2::depth_frame depth_frame, cv::Mat color_image);
};
#endif
