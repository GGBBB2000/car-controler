#include "../include/measurementPoint.hpp"

MeasurementPoint::MeasurementPoint(int frameWidth, int frameHeight) {
    for (int i = 0; i < 2; i++) {
        std::map<Direction, std::vector<cv::Point>> map;
        points.push_back(map);
    }
}

using namespace cv;
void MeasurementPoint::drawPoints(cv::Mat frame) {
    const Scalar RED = Scalar(0, 0, 255);
    const Scalar WHITE = Scalar(255, 255, 255);
    const Scalar PURPLE = Scalar(255, 0, 255);

    for (int i = 0; i < 2; i++) {
        const auto left = points[i][Direction::LEFT];
        const auto right = points[i][Direction::RIGHT];
        std::string layerString = std::to_string(i);
        for (auto p : left) {
            auto color = WHITE;
            circle(frame, p, 8, color, FILLED);
            putText(frame, layerString, cv::Point(p.x - 3, p.y), FONT_HERSHEY_SIMPLEX , 0.3, RED, 1.5, LINE_AA);
        }
        for (auto p : right) {
            auto color = WHITE;
            circle(frame, p, 8, color, FILLED);
            putText(frame, layerString, cv::Point(p.x - 3, p.y), FONT_HERSHEY_SIMPLEX , 0.3, RED, 1.5, LINE_AA);
        }
    }

    const auto center = points[0][Direction::CENTER];
    for (auto p : center) {
        auto color = RED;
        circle(frame, p, 8, color, FILLED);
    }
}

void MeasurementPoint::detectWall(rs2::depth_frame depthFrame){
    const int width = depthFrame.get_width();
    const int height = depthFrame.get_height();
    const int heightEnd = height * 2 / 3;
    std::vector<Walls> tmpWallsVec;

    for (int layer = 0; layer < 2; layer++) { 
        auto right = std::vector<cv::Point>();
        auto left = std::vector<cv::Point>();
        const float detectThreSold = this->DETECT_THRESOLD + layer / 10.0;

        //left
        int leftMax = 0.0;
        int prevLeftWallX = 0;
        if (tmpWallsVec.size() > 0) {
            prevLeftWallX = tmpWallsVec[tmpWallsVec.size() - 1].getLeftX();
        }
        for (int y = 0; y < heightEnd; y += 10) {
            Point prevPoint = cv::Point(0, y);
            for (int x = prevLeftWallX; x < width; x += 1) {
                auto distance = depthFrame.get_distance(x, y);
                if (distance == 0.0) { continue; };
                prevPoint = cv::Point(x, y);
                if (distance >= detectThreSold) {
                    if (x >= leftMax) 
                        leftMax = x;
                    left.push_back(prevPoint);
                    break;
                }
            }
        }

        //right
        int prevRightWallX = width - 1;
        int rightMin = prevRightWallX;
        if (tmpWallsVec.size() > 0) {
            prevRightWallX = tmpWallsVec[tmpWallsVec.size() - 1].getRightX();
        }
        for (int y = 0; y < heightEnd; y += 10) {
            Point prevPoint = cv::Point(0, y);
            for (int x = prevRightWallX; x >= 50; x -= 1) {
                auto distance = depthFrame.get_distance(x, y);
                if (distance == 0.0) { continue; };
                prevPoint = cv::Point(x, y);
                if (distance >= detectThreSold) {
                    if (x <= rightMin) 
                        rightMin = x;
                    right.push_back(prevPoint);
                    break;
                }
            }
        }
        auto wall = Walls(leftMax, rightMin);
        tmpWallsVec.push_back(wall);
        this->points[layer][Direction::LEFT] = left;
        this->points[layer][Direction::RIGHT] = right;
    }
    this->wallsVector = tmpWallsVec;

    auto center = std::vector<cv::Point>();
    //center
    for (int y_i = -1 ; y_i < 2; y_i++) {
        int y = height / 2 + y_i * 30;
        for (int x_i = -1; x_i < 2; x_i++) {
            int x = width / 2 + x_i * 30;
            auto distance = depthFrame.get_distance(x, y);
            if (distance <= this->DETECT_THRESOLD) {
                center.push_back(cv::Point(x, y));
            }
        }
    }
    this->points[0][Direction::CENTER] = center;
}

std::vector<Walls> MeasurementPoint::getWallsVector() {
    return this->wallsVector;
}

int MeasurementPoint::checkCenterWall() {
    return this->points[0][Direction::CENTER].size();
}
