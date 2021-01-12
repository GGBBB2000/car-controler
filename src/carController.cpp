#include "../include/carController.hpp"

CarController::CarController() {
    std::cout << "librealsense - " << RS2_API_VERSION_STR << std::endl;
    this->config.enable_stream(RS2_STREAM_DEPTH, this->FRAME_WIDTH, this->FRAME_HEIGHT, RS2_FORMAT_Z16, FPS);
    this->config.enable_stream(RS2_STREAM_COLOR, this->FRAME_WIDTH, this->FRAME_HEIGHT, RS2_FORMAT_BGR8, FPS);
    const cv::Ptr<cv::Feature2D> feature = cv::ORB::create();
    feature->detectAndCompute(this->target, cv::noArray(), this->targetKey, this->targetDescriptors);
}

void CarController::run() {
    const int fourcc = cv::VideoWriter::fourcc('m','p','4', 'v');
    auto video = cv::VideoWriter("video.mp4", fourcc, this->FPS, cv::Size(this->FRAME_WIDTH,this->FRAME_HEIGHT));
    auto color_video = cv::VideoWriter("colorvideo.mp4", fourcc, this->FPS, cv::Size(this->FRAME_WIDTH,this->FRAME_HEIGHT));
    rs2::colorizer color_map;
    rs2::pipeline pipe;

    pipe.start(this->config);

    using namespace cv;
#ifdef GUITEST
    const auto window_name = "Display Image";
    namedWindow(window_name, WINDOW_AUTOSIZE);
    while (waitKey(1) < 0 && getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0) {
#else
    while (True) {
#endif
        try {
            const rs2::frameset data = pipe.wait_for_frames(500); // Wait for next set of frames from the camera

            const rs2::frame depth = data.get_depth_frame().apply_filter(color_map);
            const rs2::depth_frame depth_raw = data.get_depth_frame();
            const rs2::frame color = data.get_color_frame();
            const Mat color_image(Size(FRAME_WIDTH, FRAME_HEIGHT), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
            const Mat depth_image(Size(FRAME_WIDTH, FRAME_HEIGHT), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);
            std::cout << color.get_data_size() << std::endl;


            Mat dest = depth_image;
            switch (state) {
                case State::RUNNING:
                case State::STOP:
                    running(depth_raw);
                    break;
                case State::TRACKING:
                    tracking(color_image, dest);
                    break;
            }
            drawMeasurementPoints(dest, depth_raw);
#ifdef GUITEST
            //drawMatches(this->target, this->targetKey, color_image, inputKey, matches, dest);
            imshow(window_name, dest);
#endif
            color_video.write(color_image);
            video.write(dest);
        } catch (rs2::error e) {
            std::cout << e.get_failed_args() << std::endl;
            std::cout << e.get_failed_function() << std::endl;
            break;
        }
    }
}

void CarController::running(rs2::depth_frame depth_raw) {
    bool center = this->points.detectObjects(Direction::CENTER, depth_raw);
    bool left = this->points.detectObjects(Direction::LEFT , depth_raw);
    bool right = this->points.detectObjects(Direction::RIGHT, depth_raw);
    if ((left && center && right) || (left && !center && right)) {
        state = State::STOP;
        throttle.setDutyCycle(0);
    } else if (!left && center && right) {
        state = State::RUNNING;
        steer.setScale(-1);
        throttle.setDutyCycle(40);
    } else if (left && center && !right) {
        state = State::RUNNING;
        steer.setScale(1);
        throttle.setDutyCycle(40);
    } else if (!left && !center && right) {
        state = State::RUNNING;
        steer.setScale(-0.5);
        throttle.setDutyCycle(40);
    } else if (left && !center && !right) {
        state = State::RUNNING;
        steer.setScale(0.5);
        throttle.setDutyCycle(40);
    } else {
        state = State::RUNNING;
        steer.setScale(-0.3);
        throttle.setDutyCycle(40);
    }
}

using namespace cv;
void CarController::tracking(Mat color_image, Mat dest) {
    const auto result = detectFeatures(color_image);
    const std::vector<KeyPoint> inputKey = result.first;
    const std::vector<std::vector<DMatch>> matches = result.second;

    if (matches.size() > 5) {
        float gravity_x = 0.0;
        float gravity_y = 0.0;
        for (std::vector<DMatch> m : matches) {
            const int index = m[0].queryIdx;
            Point point;
            point.x = inputKey[index].pt.x;
            gravity_x += inputKey[index].pt.x;
            point.y = inputKey[index].pt.y;
            gravity_y += inputKey[index].pt.y;
            circle(dest, point, 10, Scalar(0, 0, 255), FILLED);
        }
        gravity_x /= (float) matches.size();
        gravity_y /= (float) matches.size();
        circle(dest, Point(gravity_x, gravity_y), 10, Scalar(52, 235, 229), FILLED);
    }
}

std::pair<cv::Mat, cv::Mat> getColorAndDepthFrame() {
}

using namespace cv;
inline void CarController::drawMeasurementPoints(Mat dest, rs2::depth_frame depth) {
    const double center_x = FRAME_WIDTH / 2;
    const double center_y = FRAME_HEIGHT / 2;
    const auto text = std::to_string(depth.get_distance((int)center_x, (int)center_y));
    const auto end = std::chrono::system_clock::now();
    const auto time = std::chrono::system_clock::to_time_t(end);

    putText(dest, text, Point(200, 70.0), FONT_HERSHEY_SIMPLEX , 0.5, Scalar(0, 0, 255), 2, LINE_AA);
    putText(dest, std::ctime(&time), Point(0.0, 20.0) ,FONT_HERSHEY_SIMPLEX , 0.5, Scalar(0, 0, 255), 2, LINE_AA);
    std::string stateStr = ""; 
    switch (this->state) {
        case State::RUNNING:
            stateStr = "RUNNING";
            break;
        case State::TRACKING:
            stateStr = "TRACKING";
            break;
        case State::STOP:
            stateStr = "STOP";
            break;
    }
    putText(dest, stateStr, Point(0.0, 50.0) ,FONT_HERSHEY_SIMPLEX , 1.0, Scalar(255, 255, 255), 5, LINE_AA);

    this->points.drawPoints(dest, depth);
}

using namespace std;
using namespace cv;
pair<vector<KeyPoint>, vector<vector<DMatch>>> CarController::detectFeatures(cv::Mat color_image)  {
    const Ptr<Feature2D> feature = ORB::create();
    vector<KeyPoint> inputKey;
    Mat inputDescriptors;
    vector<vector<DMatch>> matches;

    const BFMatcher matcher(NORM_HAMMING2, false);
    feature->detectAndCompute(color_image, noArray(), inputKey, inputDescriptors);
    matcher.knnMatch(targetDescriptors, inputDescriptors, matches, 3);

    constexpr float ratio = 0.7f;
    vector<vector<DMatch>> processed_matches;
    for (unsigned i = 0; i < matches.size(); i++) {
        if (matches[i][0].distance < ratio * matches[i][1].distance) {
            processed_matches.push_back(matches[i]);
        }
    }

    return make_pair(inputKey, processed_matches);
}
