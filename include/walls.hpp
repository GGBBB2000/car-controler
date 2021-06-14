#ifndef INCLUDE_WALL
#define INCLUDE_WALL
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>

class Walls {
    public:
        Walls(int leftX, int rightX);
        int getInterval();
        int getCenterX();
        void draw(cv::Mat frame);
        int getLeftX(); 
        int getRightX();
    private:
        int leftX, rightX;
};

#endif
