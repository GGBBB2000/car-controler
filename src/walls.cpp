#include "../include/walls.hpp"

Walls::Walls(int leftX, int rightX) {
    this->leftX = leftX;
    this->rightX = rightX;
}

int Walls::getInterval() {
    return rightX - leftX;
}
int Walls::getCenterX() {
    return (leftX + rightX) / 2;
}

int Walls::getLeftX() {
    return leftX;
}

int Walls::getRightX() {
    return rightX;
}

using namespace cv;
void Walls::draw(cv::Mat frame) {
    const Scalar PURPLE = Scalar(255, 0, 255);
    line(frame, Point(leftX, 0), Point(leftX, frame.rows), PURPLE, 3);
    line(frame, Point(rightX, 0), Point(rightX, frame.rows), PURPLE, 3);
}
