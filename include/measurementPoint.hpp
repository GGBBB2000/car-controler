#ifndef INCLUDE_MEASUREMENTPOINT
#define INCLUDE_MEASUREMENTPOINT
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <vector>
#include <map>
#include "../include/walls.hpp"

enum Direction {
    LEFT,
    CENTER,
    RIGHT
};

class MeasurementPoint {

    public:
        MeasurementPoint();
        void drawPoints(cv::Mat frame);
        void detectWall(rs2::depth_frame depthFrame);
        std::vector<Walls> getWallsVector();
        int checkCenterWall();
    private:
        const float DETECT_THRESOLD = 0.7;
        std::vector<std::map<Direction, std::vector<cv::Point>> > points;
        std::vector<Walls> wallsVector;
};

#endif
