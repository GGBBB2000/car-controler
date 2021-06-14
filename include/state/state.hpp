#ifndef INCLUDE_STATE
#define INCLUDE_STATE
#include "/usr/local/cuda/include/cuda_runtime.h"
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
//#include "walls.hpp"
//#include "pwmController.hpp"

enum State {
    RUNNING,
    TRACKING,
    RETRY,
    STOP,
};

class Run : public StateInterface<State> {
    public:
        Run();
        void doAction();
        std::string getName();
        State getNextState();
};

class Stop : public StateInterface<State> {
    public:
        Stop();
        void doAction();
        std::string getName();
        State getNextState();
};
class Tracking : public StateInterface<State> {
    public:
        Tracking();
        void doAction();
        std::string getName();
        State getNextState();

    private:
        std::vector<cv::cuda::GpuMat> loadTemplates(const std::string templatePath);
        cv::Mat getStream(rs2::pipeline pipe);
        cv::Mat applyImageFilter(const cv::Mat color_image);
        bool searchTarget(rs2::pipeline pipe);
        void execTemplateMatching(cv::Mat gauss, std::vector<cv::cuda::GpuMat> targetMatVec);
        MeasurementPoint points = MeasurementPoint(424, 240);
};
#endif
