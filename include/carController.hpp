#ifndef INCLUDE_CAR_CONTROLLER
#define INCLUDE_CAR_CONTROLLER
#include <librealsense2/rs.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <tuple>
#include <ctime>
#include <chrono>
#include "pwmController.hpp"
#include "measurementPoint.hpp"
#define GUITEST

enum State {
    RUNNING,
    TRACKING,
    RETRY,
    STOP,
};

class CarController {
    const int FRAME_WIDTH = 424;
    const int FRAME_HEIGHT = 240;
    const int FPS = 30;
    const cv::Mat target = cv::imread("../image.jpg");
    rs2::config config;
    cv::Mat targetDescriptors; 
    std::vector<cv::KeyPoint> targetKey;
    PWM::Throttle throttle;
    PWM::Steer steer;
    State state = State::RUNNING;
    MeasurementPoint points = MeasurementPoint(FRAME_WIDTH, FRAME_HEIGHT);

    public:
        CarController();
        void run();
    private:
        void running(rs2::depth_frame depth_raw);
        void tracking(cv::Mat color_image, cv::Mat dest);
        std::pair<cv::Mat, cv::Mat> getColorAndDepthFrame();
        std::pair<std::vector<cv::KeyPoint>, std::vector<std::vector<cv::DMatch>>> detectFeatures(cv::Mat color_image) ;
        void drawMeasurementPoints(cv::Mat dest, rs2::depth_frame depth);
};

#endif // INCLUDE_CAR_CONTROLER
