#include "../include/measurementPoint.hpp"

MeasurementPoint::MeasurementPoint(int frameWidth, int frameHeight) {
    const double center_x = frameWidth / 2;
    const double center_y = frameHeight / 2;
    std::vector<cv::Point> left, center, right;

    //Left side wall
    left.push_back(cv::Point(frameWidth / 7.0, center_y - 60));
    left.push_back(cv::Point(frameWidth / 7.0, center_y - 30));
    left.push_back(cv::Point(frameWidth / 7.0, center_y));
    left.push_back(cv::Point(frameWidth / 7.0, center_y + 30));
    //left.push_back(cv::Point(frameWidth / 7.0, center_y + 60));
    left.push_back(cv::Point(frameWidth / 7.0 - 30, center_y - 60));
    left.push_back(cv::Point(frameWidth / 7.0 - 30, center_y - 30));
    left.push_back(cv::Point(frameWidth / 7.0 - 30, center_y));
    left.push_back(cv::Point(frameWidth / 7.0 - 30, center_y + 30));
    //left.push_back(cv::Point(frameWidth / 7.0 - 30, center_y + 60));

    //Right side wall
    right.push_back(cv::Point(frameWidth / 7.0 * 6.0, center_y - 60));
    right.push_back(cv::Point(frameWidth / 7.0 * 6.0, center_y - 30));
    right.push_back(cv::Point(frameWidth / 7.0 * 6.0, center_y));
    right.push_back(cv::Point(frameWidth / 7.0 * 6.0, center_y + 30));
    //right.push_back(cv::Point(frameWidth / 7.0 * 6.0, center_y + 60));
    right.push_back(cv::Point(frameWidth / 7.0 * 6.0 + 30 , center_y - 60));
    right.push_back(cv::Point(frameWidth / 7.0 * 6.0 + 30 , center_y - 30));
    right.push_back(cv::Point(frameWidth / 7.0 * 6.0 + 30 , center_y));
    right.push_back(cv::Point(frameWidth / 7.0 * 6.0 + 30 , center_y + 30));
    //right.push_back(cv::Point(frameWidth / 7.0 * 6.0 + 30 , center_y + 60));

    //Center
    center.push_back(cv::Point(center_x - 30, center_y - 30));
    center.push_back(cv::Point(center_x - 30, center_y + 30 ));
    center.push_back(cv::Point(center_x - 30, center_y));
    center.push_back(cv::Point(center_x, center_y - 30));
    center.push_back(cv::Point(center_x, center_y));
    center.push_back(cv::Point(center_x, center_y + 30));
    center.push_back(cv::Point(center_x + 30, center_y));
    center.push_back(cv::Point(center_x + 30, center_y - 30));
    center.push_back(cv::Point(center_x + 30, center_y + 30));

    points.insert(std::make_pair(Direction::LEFT, left));
    points.insert(std::make_pair(Direction::CENTER, center));
    points.insert(std::make_pair(Direction::RIGHT, right));
}

using namespace cv;
void MeasurementPoint::drawPoints(cv::Mat frame, rs2::depth_frame depthFrame) {
    const Scalar RED = Scalar(0, 0, 255);
    const Scalar WHITE = Scalar(255, 255, 255);
    const auto left = points[Direction::LEFT];
    const auto center = points[Direction::CENTER];
    const auto right = points[Direction::RIGHT];
    for (auto p : left) {
        auto color = RED;
        auto distance = depthFrame.get_distance(p.x, p.y);
        if (distance < 0.5 && distance != 0.0) {
            color = WHITE;
        }
        circle(frame, p, 8, color, FILLED);
    }
    for (auto p : center) {
        auto color = RED;
        auto distance = depthFrame.get_distance(p.x, p.y);
        if (distance < 0.5 && distance != 0.0) {
            color = WHITE;
        }
        circle(frame, p, 8, color, FILLED);
    }
    for (auto p : right) {
        auto color = RED;
        auto distance = depthFrame.get_distance(p.x, p.y);
        if (distance < 0.5 && distance != 0.0) {
            color = WHITE;
        }
        circle(frame, p, 8, color, FILLED);
    }
}

bool MeasurementPoint::detectObjects(Direction dir, rs2::depth_frame depthFrame){
    auto target = this->points[dir];
    for (auto p : target) {
        auto distance = depthFrame.get_distance(p.x, p.y);
        if (distance < 0.7 && distance != 0.0) {
            return true;
        }
    }
    return false;

}
