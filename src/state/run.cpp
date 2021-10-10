#include "../../include/state/state.hpp"
#include "../../include/pwmController.hpp"
#include "../../include/measurementPoint.hpp"
#include <opencv2/aruco.hpp>

Run::Run(std::shared_ptr<StreamManager> st) {
    
}


#include <thread>
#include <chrono>
void Run::rotate() {
    Steer steer;
    Throttle throttle;
    using namespace std::chrono;
    using namespace std;

    for (int i = 0; i < 10; i++) {
	steer.setScale(-1.0);
	this_thread::sleep_for(milliseconds(1000));

	throttle.setScale(-0.5);
	this_thread::sleep_for(milliseconds(500));

	steer.setScale(1.0);
	throttle.setScale(0.0);
	this_thread::sleep_for(milliseconds(1000));

	throttle.setScale(0.5);
	this_thread::sleep_for(milliseconds(500));

	steer.setScale(1.0);
	throttle.setScale(0.0);
	this_thread::sleep_for(milliseconds(1000));
    }
    this->isDetectingMarker = true;
    throttle.setScale(0.8);
}

#include <chrono>
void Run::doAction() {
    //std::this_thread::sleep_for(seconds(30));
    Steer steer;
    Throttle throttle;

    const int FRAME_WIDTH = 848;
    const int FRAME_HEIGHT = 480;

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    const auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    rs2::config config;
    config.enable_stream(RS2_STREAM_COLOR, FRAME_WIDTH, FRAME_HEIGHT, RS2_FORMAT_BGR8, 30);
    config.enable_stream(RS2_STREAM_DEPTH, FRAME_WIDTH, FRAME_HEIGHT, RS2_FORMAT_Z16, 30);
    rs2::pipeline pipe;
    rs2::align align_to(RS2_STREAM_COLOR);
    pipe.start(config);

    rs2::colorizer color_map;
    throttle.setScale(0.8);

    using namespace cv;
    try {
	while(cv::waitKey(1) < 0) {
	    const rs2::frameset frames = align_to.process(pipe.wait_for_frames(1000));
	    const rs2::frame color_frame = frames.get_color_frame();
	    const rs2::frame depth_frame = frames.get_depth_frame().apply_filter(color_map);
	    const rs2::depth_frame depth_raw = frames.get_depth_frame();
	    auto mesurementPoint = MeasurementPoint();
	    Mat color_image(Size(FRAME_WIDTH, FRAME_HEIGHT), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
	    Mat depth_image(Size(FRAME_WIDTH, FRAME_HEIGHT), CV_8UC3, (void*)depth_frame.get_data(), Mat::AUTO_STEP);
	    

	    mesurementPoint.detectWall(depth_raw);
	    const auto walls = mesurementPoint.getWallsVector();

	    auto centerAvg = 0.0;
	    {
		auto centerSum = 0;
		for (auto wall: walls) {
		    centerSum += wall.getCenterX();
		}
		centerAvg = centerSum / (double)walls.size();
	    }
	    const double scale = (centerAvg -  FRAME_WIDTH / 2.0) / (double)(FRAME_WIDTH / 2.0) * 5.0;

	    steer.setScale(scale);
	    if (this->isDetectingMarker) {
		aruco::detectMarkers(color_image, dictionary, markerCorners, markerIds);
		if (markerCorners.size() > 0) {
		    double x_sum = 0;
		    double y_sum = 0;
		    for (auto p: markerCorners[0]) {
			y_sum += p.y;
		    }
		    const int markerX = x_sum / 4.0;
		    const int markerY = y_sum / 4.0;
		    const float distance = depth_raw.get_distance(markerX, markerY); 
		    if (distance != 0 && distance < 1.5) {
			//break; // goto tracking
		    }
		}
	    } else if(mesurementPoint.checkCenterWall() > 30) {
		throttle.setScale(0.0);
		steer.setScale(0.0);
		rotate();
	    }


	    mesurementPoint.drawPoints(color_image);
	    Mat arr[] = {
		color_image,
		depth_image
	    };
	    Mat out;// = color_image;
	    hconcat(arr, 2, out);

	    imshow("", out);
	}
    } catch (rs2::error e) {
        std::cout << e.get_failed_args() << std::endl;
        std::cout << e.get_failed_function() << std::endl;
    }
    pipe.stop();
    steer.setScale(0);
    throttle.setScale(0);
}

std::string Run::getName() {
    return "RUN";
}

State Run::getNextState() {
    return State::TRACKING;
}
