#include "../../include/state/state.hpp"

Run::Run() {
}

void Run::doAction() {
    const int fourcc = cv::VideoWriter::fourcc('m','p','4', 'v');
    rs2::colorizer color_map;
    rs2::pipeline pipe;
    rs2::config config;
    config.enable_stream(RS2_STREAM_COLOR, 424, 240, RS2_FORMAT_BGR8, 60);
    pipe.start(config);

    using namespace cv;
    namedWindow("hoge", WINDOW_AUTOSIZE);
    try {
        while (waitKey(1) < 0 && getWindowProperty("hoge", WND_PROP_AUTOSIZE) >= 0) {
            const rs2::frameset data = pipe.wait_for_frames(500); // Wait for next set of frames from the camera
            const rs2::frame color = data.get_color_frame();
            const Mat color_image(Size(424, 240), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);

            imshow("hoge", color_image);
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
