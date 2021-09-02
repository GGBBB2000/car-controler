#include "../../include/state/state.hpp"
#include "../../include/pwmController.hpp"
#include <opencv2/aruco.hpp>

Tracking::Tracking(std::shared_ptr<StreamManager> st) {
    this->streamManager = st;
}

#include <chrono>
#define CALC_TIME(statement) auto time_start = std::chrono::system_clock::now(); \
                                               statement ;  \
                                               auto time_end = std::chrono::system_clock::now();

#define PRINT_DURATION(statement) CALC_TIME(statement) \
    auto dur = time_end - time_start; \
    auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count(); \
    std::cout << msec << "msec" << std::endl; \

cv::Mat markerImage;
std::vector<int> markerIds;
std::vector<std::vector<cv::Point2f>> markerCorners, rejected;
cv::Ptr<cv::aruco::DetectorParameters> parameters;
auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

bool Tracking::steerControll(rs2::depth_frame depth_frame, cv::Mat color_image) {
    Steer steer;
    Throttle throttle;
    using namespace cv;
    aruco::detectMarkers(color_image, dictionary, markerCorners, markerIds);

    //line(color_image, Point(color_image.cols / 2 - 200, 100), Point(color_image.cols / 2 + 200, 100), Scalar(255, 255, 255), 3);
    for (auto marker: markerCorners) {
	auto xSum = 0;
	auto ySum = 0;
	for (auto points: marker) {
	    xSum += points.x;
	    ySum += points.y;
	}
	const Point leftPoint2D((marker[0].x + marker[3].x) / 2.0, (marker[0].y + marker[3].y) / 2.0);
	const Point rightPoint2D((marker[1].x + marker[2].x) / 2.0, (marker[1].y + marker[2].y) / 2.0);
	const Point3f leftPoint3D(leftPoint2D.x, leftPoint2D.y, depth_frame.get_distance(leftPoint2D.x, leftPoint2D.y));
	const Point3f rightPoint3D(rightPoint2D.x, rightPoint2D.y, depth_frame.get_distance(rightPoint2D.x, rightPoint2D.y));
	const double markerCenterX = xSum / 4.0;
	const double markerCenterY = ySum / 4.0;
	const double distance = depth_frame.get_distance(markerCenterX, markerCenterY);
	const double steerScaleRate = 3.0;
	const double realToVirtualScale = (rightPoint3D.x - leftPoint3D.x) / 0.10; // 0.15 marker no ookisa
	const double alpha = 1.0 * (rightPoint3D.z - leftPoint3D.z) * realToVirtualScale; // 3.0 tomarubasho
	const double xp = markerCenterX + alpha;
	const double yp = ((rightPoint2D.y - leftPoint2D.y) / (rightPoint2D.x - leftPoint2D.x)) * (xp - rightPoint2D.x) + rightPoint2D.y;
	double steerScale = 0;
	
	if (abs(xp - markerCenterX) < 30) {
	    steerScale = markerCenterX / (double)color_image.cols * (steerScaleRate * 2.0) - steerScaleRate;
	} else {
	    steerScale = xp / (double)color_image.cols * (steerScaleRate * 2.0) - steerScaleRate;
	}

	//const double indicatorX = markerCenterX / (double)color_image.cols * 400;

	if (distance < 0.15) {
	    throttle.setScale(0);
	    return true;
	}
	steer.setScale(steerScale);

	circle(color_image, Point(xSum / 4, ySum / 4), 10, Scalar(0, 0, 255), FILLED);
	putText(color_image, "C", Point(xSum / 4 - 20, ySum / 4 - 20), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 4);
	circle(color_image, Point(leftPoint2D.x, leftPoint2D.y), 10, Scalar(0, 0, 255), FILLED);
	putText(color_image, "A", Point(leftPoint2D.x - 20, leftPoint2D.y - 20), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 4);
	circle(color_image, Point(rightPoint2D.x, rightPoint2D.y), 10, Scalar(0, 0, 255), FILLED);
	putText(color_image, "B", Point(rightPoint2D.x - 20, rightPoint2D.y - 20), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 4);
	circle(color_image, Point(xp, yp), 10, Scalar(0, 255, 0), FILLED);
	putText(color_image, "P", Point(xp - 20, yp - 20), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 4);
	putText(color_image, std::to_string(distance), Point(100, 100), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 4);
	



	//putText(color_image, std::to_string(steerScale), Point(color_image.cols / 2, 80), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 0), 4);
	//putText(color_image, "distance:" + std::to_string(distance), Point(color_image.cols / 2, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 0), 4);
    }
    return false;
}

const int FRAME_WIDTH = 848;
const int FRAME_HEIGHT = 480;
void Tracking::doAction() {
    using namespace cv;
    auto start = std::chrono::system_clock::now();

    rs2::config config;
    config.enable_stream(RS2_STREAM_COLOR, FRAME_WIDTH, FRAME_HEIGHT, RS2_FORMAT_BGR8, 30);
    config.enable_stream(RS2_STREAM_DEPTH, FRAME_WIDTH, FRAME_HEIGHT, RS2_FORMAT_Z16, 30);
    rs2::pipeline pipe;
    rs2::align align_to(RS2_STREAM_COLOR);
    pipe.start(config);

    rs2::colorizer color_map;
    auto count = 0;
    try {
        while (waitKey(1) < 0) {
        //while (true) {
	    const rs2::frameset frames = align_to.process(pipe.wait_for_frames(1000));
	    const rs2::frame color_frame = frames.get_color_frame();
	    const rs2::frame depth_frame = frames.get_depth_frame().apply_filter(color_map);
	    const rs2::frame depth_raw = frames.get_depth_frame();
	    Mat color_image(Size(FRAME_WIDTH, FRAME_HEIGHT), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
	    Mat depth_image(Size(FRAME_WIDTH, FRAME_HEIGHT), CV_8UC3, (void*)depth_frame.get_data(), Mat::AUTO_STEP);

	    const bool isArrived = this->steerControll(depth_raw, color_image); 
	    if (isArrived) {
	        break;
	    }

            // fps calc
            count++;
            auto end = std::chrono::system_clock::now();
            auto dur = end - start;
            auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();
            double fps = (count / (double)msec) * 1000.0;
            if (msec > 1000) {
                start = std::chrono::system_clock::now();
                count = 0;
            }
            putText(color_image, std::to_string(fps) + "fps" , Point(30, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 0), 4);
            // fps calc end

	    Mat arr[] = {
		color_image,
		depth_image
	    };
	    Mat out = depth_image;
	    //hconcat(arr, 2, out);
	    imshow("result", out);
        }
    } catch (rs2::error e) {
        std::cout << e.get_failed_args() << std::endl;
        std::cout << e.get_failed_function() << std::endl;
    } 
    pipe.stop();
}

std::string Tracking::getName() {
    return "TRACKING";
}

State Tracking::getNextState() {
    return State::STOP;
}
