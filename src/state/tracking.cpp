#include "../../include/state/state.hpp"
#include "../../include/pwmController.hpp"
#include <opencv2/aruco.hpp>

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

/*
 *
            // connectedComponentsWithStats
            //Mat labels, stats, centroids;
            //int nLabels = connectedComponentsWithStats(gauss, labels, stats, centroids, 8, CV_16UC1);

            //for (int i = 1; i < nLabels; i++) {
            //    int *param = stats.ptr<int>(i);

            //    int x = param[ConnectedComponentsTypes::CC_STAT_LEFT];
            //    int y = param[ConnectedComponentsTypes::CC_STAT_TOP];
            //    int height = param[ConnectedComponentsTypes::CC_STAT_HEIGHT];
            //    int width = param[ConnectedComponentsTypes::CC_STAT_WIDTH];
            //    int area = height * width;
            //    //if (area < 500 || area > 10000) {
            //    //    rectangle(gauss, Rect(x, y, width, height), Scalar(0, 0, 0), FILLED);
            //    //}
            //}
 */

std::vector<cv::cuda::GpuMat> Tracking::loadTemplates(const std::string templatePath = "../light.png") {
    std::vector<cv::cuda::GpuMat> targetMatVec;
    const cv::Mat lightTmp = cv::imread(templatePath);
    double size = 1 / 5.5;
    int count = 0;

    while (size < 1.5) {
        {
            cv::Mat target;
            std::cout << "loading...:" << count << std::endl;
            cv::cuda::GpuMat gpuIn, resized, gpuOut;
            gpuIn.upload(lightTmp);
            cv::cuda::resize(gpuIn, resized, cv::Size(), size, size);
            cv::cuda::cvtColor(resized, gpuOut, cv::COLOR_BGR2GRAY);
            targetMatVec.push_back(gpuOut);
            size += 0.01;
        }
        count++;
    }
    return targetMatVec;
}

//TODO  何かフレーム取得用クラスを作る
cv::Mat Tracking::getStream(rs2::pipeline pipe) {
    const rs2::frameset data = pipe.wait_for_frames(500); // Wait for next set of frames from the camera
    //iconst rs2::frame infrared = data.get_infrared_frame();
    //const cv::Mat color_image(cv::Size(424, 240), CV_8UC1, (void*)infrared.get_data(), cv::Mat::AUTO_STEP);
    const rs2::frame color = data.get_color_frame();
    const cv::Mat color_image(cv::Size(424, 240), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
    return color_image;
}

bool Tracking::searchTarget(rs2::pipeline pipe) {
    const rs2::frameset data = pipe.wait_for_frames(500); // Wait for next set of frames from the camera
    const rs2::frame depth = data.get_depth_frame();
    const rs2::depth_frame depth_raw = data.get_depth_frame();
    const cv::Mat depth_image(cv::Size(424, 240), CV_8UC3, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
    return false;
}


cv::Mat Tracking::applyImageFilter(const cv::Mat color_image) {
    cv::Mat gauss; 
    {
        cv::cuda::Stream s;
        cv::Mat binImage;
        cv::cuda::GpuMat gpuColorImage, gpuOut;
        gpuColorImage.upload(color_image);
        cv::cuda::threshold(gpuColorImage, gpuOut, 254, 255, cv::THRESH_BINARY, s);
        gpuOut.download(binImage, s);
        cv::GaussianBlur(binImage, gauss, cv::Size(13, 13), 1.0);
    }
    return gauss;
}

auto steer = Steer();
int lastMatchedIndex = 0;
const size_t windowSize = 5;
using namespace cv;
void Tracking::execTemplateMatching(cv::Mat gauss, std::vector<cuda::GpuMat> targetMatVec) {
    Ptr<cuda::TemplateMatching> ptr = cuda::createTemplateMatching(CV_8U, TM_CCOEFF_NORMED);

    cv::rectangle(gauss, cv::Point(0, 0), cv::Point(targetMatVec[lastMatchedIndex].cols, targetMatVec[lastMatchedIndex].rows), cv::Scalar(255,255,0), 1);

    #pragma omp for
    for (int i = 0; i < windowSize; i++) {
        if (lastMatchedIndex + i < targetMatVec.size()) {

            auto target = targetMatVec[lastMatchedIndex + i];
            cuda::Stream st;

            //Mat templateOut;
            cuda::GpuMat gpuIn, targetGpu, gpuOut;
            gpuIn.upload(gauss);
            ptr->match(gpuIn, target, gpuOut);

            cv::Rect roi_rect(0, 0, target.cols, target.rows);
            Point max_pt;
            double maxVal;
            cuda::minMaxLoc(gpuOut, NULL, &maxVal, NULL, &max_pt);

            if (maxVal > 0.6) {
                //std::cout << max_pt.x << " " << max_pt.y <<  "\n";
                roi_rect.x = max_pt.x;
                roi_rect.y = max_pt.y;
                //std::cout << roi_rect.width << " " << roi_rect.height << std::endl;
                //std::cout << maxVal << "\n"; 

                #pragma omp critical
                {
                    rectangle(gauss, roi_rect, Scalar(255,0,0), 3);
                    if (lastMatchedIndex < i + lastMatchedIndex) {
                        //Steer Controll
                        const int flameWidth = gauss.cols;
                        const int centerX = flameWidth / 2; 
                        if (max_pt.x < centerX) { // ターゲットが画面左側の時
                            const int diff = (centerX - max_pt.x);
                            const double steerVal = diff / (flameWidth / 2.0);
                            const double coef = 5.0;
                            //steer.setScale(-steerVal * coef);
                            //std::cout << -steerVal * coef << "\n";
                        } else {
                            const int diff = (max_pt.x - centerX);
                            const double steerVal = diff / (flameWidth / 2.0);
                            const double coef = 3.0;
                            //steer.setScale(steerVal * coef);
                            //std::cout << steerVal * coef << "\n";
                        }

                        std::cout << 1 / 5.5 + (lastMatchedIndex / 100.0) << "\n";
                        //Steer Controll

                        lastMatchedIndex = i + lastMatchedIndex;
                    }
                }
            }
        }
    }
}

void Tracking::doAction() {
    std::vector<cv::cuda::GpuMat> targetMatVec = this->loadTemplates();

    rs2::pipeline pipe;
    rs2::config config;
    config.enable_stream(RS2_STREAM_INFRARED, 1, 424, 240, RS2_FORMAT_Y8, 30);
    config.enable_stream(RS2_STREAM_DEPTH, 424, 240, RS2_FORMAT_Z16, 30);
    config.enable_stream(RS2_STREAM_COLOR, 424, 240, RS2_FORMAT_BGR8, 30);
    //config.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);
    pipe.start(config);

    using namespace cv;
    const int fourcc = VideoWriter::fourcc('m','p','4', 'v');
    auto bin = VideoWriter("bin.mp4", fourcc, 30, Size(424, 240), false);
    auto processed = VideoWriter("process.mp4", fourcc, 30, Size(424, 240), false);
    //namedWindow("hoge", WINDOW_AUTOSIZE);
    auto start = std::chrono::system_clock::now();

    Mat markerImage;
    std::vector<int> markerIds;
    std::vector<std::vector<Point2f>> markerCorners, rejected;
    Ptr<aruco::DetectorParameters> parameters;
    auto dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
    //aruco::drawMarker(dictionary, 24, 200, markerImage, 1);
    //imshow("marker", markerImage);

    auto count = 0;
    try {
        while (waitKey(1) < 0) {
        //while (true) {
            const Mat color_image = this->getStream(pipe);
            Mat out;
            aruco::detectMarkers(color_image, dictionary, markerCorners, markerIds);
            if (markerIds.size() > 0) {
                aruco::drawDetectedMarkers(out, markerCorners, markerIds);
                imshow("hoge", out);
            } else {
                imshow("hoge", color_image);
            }
            std::cout << markerIds.size() << std::endl;
            //this->searchTarget(pipe);
            //cv::Mat gauss = this->applyImageFilter(color_image);

            //this->execTemplateMatching(gauss, targetMatVec);
            //bin.write(gauss);
            //if (lastMatchedIndex > targetMatVec.size() - 5) {
            //    break;
            //}

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
