#ifndef INCLUDE_CAR_MODEL
#define INCLUDE_CAR_MODEL
#include <librealsense2/rs.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <tuple>
#include <ctime>
#include <chrono>
#include "measurementPoint.hpp"
#include "walls.hpp"
#include "pwmController.hpp"
#include "../include/state/state.hpp"

class CarModel {
    const int FRAME_WIDTH = 424;
    const int FRAME_HEIGHT = 240;
    const int FPS = 30;
    rs2::config config;
    Steer steer;
    Throttle throttle;
    //State state;
    cv::Mat targetDescriptors; 
    std::vector<cv::KeyPoint> targetKey;
    MeasurementPoint points = MeasurementPoint(FRAME_WIDTH, FRAME_HEIGHT);
    std::map<State, std::shared_ptr<StateInterface<State>>> stateMap;

    public:
        CarModel();
        void run();
    private:
        void running(rs2::depth_frame depth_raw);
        void tracking(cv::Mat color_image, cv::Mat dest);
        std::pair<cv::Mat, cv::Mat> getColorAndDepthFrame();
        std::pair<std::vector<cv::KeyPoint>, std::vector<std::vector<cv::DMatch>>> detectFeatures(cv::Mat color_image) ;
        void drawMeasurementPoints(cv::Mat dest, rs2::depth_frame depth);
};

#endif // INCLUDE_CAR_MODEL
