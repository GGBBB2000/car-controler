#include "../include/streamManager.hpp"

const int WIDTH = 640;
const int HEIGHT = 480;
const int FPS = 30;

StreamManager::StreamManager() {
    rs2::config config;
    config.enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16, FPS);
    config.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_BGR8, FPS);
    this->pipe.start(config);
}

cv::Mat StreamManager::getFrameAsMat(rs2_stream type) {
    const rs2::frameset data = this->pipe.wait_for_frames(500);
    rs2::frame f;
    cv::Mat m;
    switch(type) {
	case RS2_STREAM_DEPTH:
	    f = data.get_depth_frame();
	    m = cv::Mat(cv::Size(WIDTH, HEIGHT), CV_8UC1, (void*)f.get_data(), cv::Mat::AUTO_STEP);
	    return m;
	case RS2_STREAM_COLOR:
	    f = data.get_color_frame();
	    m = cv::Mat(cv::Size(WIDTH, HEIGHT), CV_8UC3, (void*)f.get_data(), cv::Mat::AUTO_STEP);
	    return m;
    }
    exit(1); // TODO return exception
}

rs2::frame StreamManager::getFrameAsRS2Frame(rs2_stream type) {
    const rs2::frameset data = this->pipe.wait_for_frames(500);
    switch(type) {
	case RS2_STREAM_DEPTH:
	    return data.get_depth_frame();

	case RS2_STREAM_COLOR:
	    return data.get_color_frame();
    }
    exit(1); // TODO return exception
}

#include <iostream>
StreamManager::~StreamManager() {
    std::cout << "hoge" << std::endl;
    pipe.stop();
}
