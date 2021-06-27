#include "../include/carModel.hpp"

CarModel::CarModel() {
    std::cout << "Using librealsense - " << RS2_API_VERSION_STR << std::endl;
    std::shared_ptr<StreamManager> stPtr = std::make_shared<StreamManager>();
    stateMap.insert(std::make_pair(State::RUNNING, std::make_shared<Run>(stPtr)));
    stateMap.insert(std::make_pair(State::STOP, std::make_shared<Stop>(stPtr)));
    stateMap.insert(std::make_pair(State::TRACKING, std::make_shared<Tracking>(stPtr)));
}

void CarModel::run() {
    std::shared_ptr<StateInterface<State>> state;
    auto nextState = State::TRACKING;//TODO あとで初期状態を作る
    while (nextState != State::STOP) {
        state =  stateMap[nextState];
        state->doAction();
        std::cout << state->getName() << std::endl;
        nextState = state->getNextState();
    }


    //const int fourcc = cv::VideoWriter::fourcc('m','p','4', 'v');
    //auto video = cv::VideoWriter("video.mp4", fourcc, this->FPS, cv::Size(this->FRAME_WIDTH,this->FRAME_HEIGHT));
    //auto color_video = cv::VideoWriter("colorvideo.mp4", fourcc, this->FPS, cv::Size(this->FRAME_WIDTH,this->FRAME_HEIGHT));
    //rs2::colorizer color_map;
    //rs2::pipeline pipe;
    /*
     * 0: fill from left
     * 1: farest from around
     * 2: nearest from around
     */
    //rs2::hole_filling_filter holefilling(2);

    //cv::VideoCapture cap(0);
    //cap.set(cv::CAP_PROP_FPS, this->FPS);
    //auto cap_video = cv::VideoWriter("capture.mp4", fourcc, this->FPS, cv::Size(640, 480));

//    pipe.start(this->config);
//    this->state = State::TRACKING;
//
//    using namespace cv;
//#ifdef GUITEST
//    const auto window_name = "Display Image";
//    namedWindow(window_name, WINDOW_AUTOSIZE);
//    while (waitKey(1) < 0 && getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0) {
//#else
//    while (True) {
//#endif
//        try {
//            const rs2::frameset data = pipe.wait_for_frames(500); // Wait for next set of frames from the camera
//
//            const rs2::frame depth = /*holefilling.process(*/data.get_depth_frame()/*)*/.apply_filter(color_map);
//            const rs2::depth_frame depth_raw = /*holefilling.process(*/data.get_depth_frame()/*)*/;
//            const rs2::frame color = data.get_color_frame();
//            const Mat color_image(Size(FRAME_WIDTH, FRAME_HEIGHT), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
//            const Mat depth_image(Size(FRAME_WIDTH, FRAME_HEIGHT), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);
//
//            //Mat cap_frame_tmp, cap_frame;
//            //cap.read(cap_frame_tmp);
//            //cv::rotate(cap_frame_tmp, cap_frame, cv::ROTATE_90_COUNTERCLOCKWISE);
//
//            //std::cout << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
//            //std::cout << cap.get(cv::CAP_PROP_FRAME_WIDTH) << std::endl;
//
//
//            Mat dest = depth_image;
//            switch (state) {
//                case State::RUNNING:
//                    running(depth_raw);
//                    break;
//                case State::STOP:
//                    break;
//                case State::TRACKING:
//                    tracking(color_image, dest);
//                    break;
//            }
//            drawMeasurementPoints(dest, depth_raw);
//#ifdef GUITEST
//            //drawMatches(this->target, this->targetKey, color_image, inputKey, matches, dest);
//            imshow(window_name, dest);
//            imshow("hoge", color_image);
//#endif
//            color_video.write(color_image);
//            video.write(dest);
//            //cap_video.write(cap_frame);
//        } catch (rs2::error e) {
//            std::cout << e.get_failed_args() << std::endl;
//            std::cout << e.get_failed_function() << std::endl;
//            break;
//        }
//    }
//    this->throttle.setScale(0.0);
//    this->steer.setScale(0.0);
}

#include <cmath>
void CarModel::running(rs2::depth_frame depth_raw) {
    this->points.detectWall(depth_raw);
    auto walls = this->points.getWallsVector();
    const int wallsCenter = walls[0].getCenterX();

    const float middleX = this->FRAME_WIDTH / 2.0;
    const float distanceFromCenter = std::abs(wallsCenter - middleX);
    const float distanceRatio = distanceFromCenter / middleX;
    float steerScale = distanceRatio;
    if (wallsCenter < middleX) {
        steerScale *= -1;
    }

    if (this->points.checkCenterWall() > 6) {
        //this->state = State::STOP;
        this->throttle.setScale(0.0);
        this->steer.setScale(0.0);
        return;
    }


    this->throttle.setScale(1.0);
    this->steer.setScale(steerScale);
}

using namespace cv;
void CarModel::tracking(Mat color_image, Mat dest) {
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

using namespace cv;
inline void CarModel::drawMeasurementPoints(Mat dest, rs2::depth_frame depth) {
    const double center_x = FRAME_WIDTH / 2;
    const double center_y = FRAME_HEIGHT / 2;
    const auto text = std::to_string(depth.get_distance((int)center_x, (int)center_y));
    const auto end = std::chrono::system_clock::now();
    const auto time = std::chrono::system_clock::to_time_t(end);
    const std::vector<Walls> walls = this->points.getWallsVector();

    this->points.drawPoints(dest);
    for (auto w: walls) {
        w.draw(dest);
    }
    putText(dest, text, Point(200, 70.0), FONT_HERSHEY_SIMPLEX , 0.5, Scalar(0, 0, 255), 2, LINE_AA);
    putText(dest, std::ctime(&time), Point(0.0, 20.0) ,FONT_HERSHEY_SIMPLEX , 0.5, Scalar(0, 0, 255), 2, LINE_AA);
    std::string stateStr = ""; 
    putText(dest, stateStr, Point(0.0, 50.0) ,FONT_HERSHEY_SIMPLEX , 1.0, Scalar(255, 255, 255), 5, LINE_AA);

}

using namespace std;
using namespace cv;
pair<vector<KeyPoint>, vector<vector<DMatch>>> CarModel::detectFeatures(cv::Mat color_image)  {
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
