#include "../../include/state/state.hpp"
#include "../../include/pwmController.hpp"
#include "../../include/measurementPoint.hpp"

Run::Run(std::shared_ptr<StreamManager> st) {
}


void Run::doAction() {
    const int FRAME_WIDTH = 848;
    const int FRAME_HEIGHT = 480;

    rs2::config config;
    config.enable_stream(RS2_STREAM_COLOR, FRAME_WIDTH, FRAME_HEIGHT, RS2_FORMAT_BGR8, 30);
    config.enable_stream(RS2_STREAM_DEPTH, FRAME_WIDTH, FRAME_HEIGHT, RS2_FORMAT_Z16, 30);
    rs2::pipeline pipe;
    rs2::align align_to(RS2_STREAM_COLOR);
    pipe.start(config);

    rs2::colorizer color_map;

    using namespace cv;
    try {
	while(cv::waitKey(1) < 0) {
	    const rs2::frameset frames = align_to.process(pipe.wait_for_frames(1000));
	    const rs2::frame color_frame = frames.get_color_frame();
	    const rs2::frame depth_frame = frames.get_depth_frame().apply_filter(color_map);
	    const rs2::frame depth_raw = frames.get_depth_frame();
	    auto mesurementPoint = MeasurementPoint();
	    Mat color_image(Size(FRAME_WIDTH, FRAME_HEIGHT), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
	    Mat depth_image(Size(FRAME_WIDTH, FRAME_HEIGHT), CV_8UC3, (void*)depth_frame.get_data(), Mat::AUTO_STEP);

	    mesurementPoint.detectWall(depth_raw);
	    mesurementPoint.drawPoints(depth_image);

	    imshow("", depth_image);
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
