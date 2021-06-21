#include "../../include/state/state.hpp"
#include "../../include/pwmController.hpp"
#include <opencv2/aruco.hpp>
#include <opencv2/gapi/gmat.hpp>

Tracking::Tracking() {
}

#include <chrono>
#define CALC_TIME(statement) auto time_start = std::chrono::system_clock::now(); \
                                               statement ;  \
                                               auto time_end = std::chrono::system_clock::now();

#define PRINT_DURATION(statement) CALC_TIME(statement) \
    auto dur = time_end - time_start; \
    auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count(); \
    std::cout << msec << "msec" << std::endl; \

//TODO  何かフレーム取得用クラスを作る
cv::Mat Tracking::getStream(rs2::pipeline pipe) {
    const rs2::frameset data = pipe.wait_for_frames(500); // Wait for next set of frames from the camera
    const rs2::frame color = data.get_color_frame();
    const cv::Mat color_image(cv::Size(640, 480), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
    return color_image;
}


auto steer = Steer();

void Tracking::doAction() {
    rs2::pipeline pipe;
    rs2::config config;
    config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    //config.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);
    pipe.start(config);

    using namespace cv;
    const int fourcc = VideoWriter::fourcc('m','p','4', 'v');
    auto bin = VideoWriter("bin.mp4", fourcc, 30, Size(640, 480), false);
    auto processed = VideoWriter("process.mp4", fourcc, 30, Size(640, 480), false);
    auto start = std::chrono::system_clock::now();

    Mat markerImage;
    std::vector<int> markerIds;
    std::vector<std::vector<Point2f>> markerCorners, rejected;
    Ptr<aruco::DetectorParameters> parameters;
    auto dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);

    auto count = 0;
    try {
        while (waitKey(1) < 0) {
        //while (true) {
            const Mat color_image = this->getStream(pipe);
            Mat out;
	    color_image.copyTo(out);
            aruco::detectMarkers(color_image, dictionary, markerCorners, markerIds);

            if (markerIds.size() > 0) {
                aruco::drawDetectedMarkers(out, markerCorners, markerIds);
                imshow("hoge", out);
            } else {
                imshow("hoge", color_image);
            }
            std::cout << markerIds.size() << std::endl;

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
            //putText(gauss, std::to_string(fps) + "fps" , Point(30, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 0), 4);
            // fps calc end

            //imshow("hoge", gauss);
        }
    } catch (rs2::error e) {
        std::cout << e.get_failed_args() << std::endl;
        std::cout << e.get_failed_function() << std::endl;
    }
}

std::string Tracking::getName() {
    return "TRACKING";
}

State Tracking::getNextState() {
    return State::STOP;
}
