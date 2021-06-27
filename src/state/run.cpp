#include "../../include/state/state.hpp"

Run::Run(std::shared_ptr<StreamManager> st) {
    this->streamManager = st;
}

void Run::doAction() {
    const int fourcc = cv::VideoWriter::fourcc('m','p','4', 'v');

    using namespace cv;
    try {
	while(cv::waitKey(1) < 0) {
	    cv::imshow("hoge", streamManager->getFrameAsMat(RS2_STREAM_COLOR));
	}
    } catch (rs2::error e) {
        std::cout << e.get_failed_args() << std::endl;
        std::cout << e.get_failed_function() << std::endl;
    }
}

std::string Run::getName() {
    return "RUN";
}

State Run::getNextState() {
    return State::STOP;
}
