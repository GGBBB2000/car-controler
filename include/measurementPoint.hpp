#ifndef INCLUDE_MEASUREMENTPOINT
#define INCLUDE_MEASUREMENTPOINT
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <vector>
#include <map>

enum Direction {
    LEFT,
    CENTER,
    RIGHT
};

class MeasurementPoint {

    public:
        MeasurementPoint(int frameWidth, int frameHeight);
        void drawPoints(cv::Mat frame, rs2::depth_frame depthFrame);
        bool detectObjects(Direction dir, rs2::depth_frame depthFrame);
    private:
        std::map<Direction, std::vector<cv::Point>> points;
};

#endif
