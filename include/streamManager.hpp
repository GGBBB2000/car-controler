#ifndef INCLUDE_STMANAGER
#define INCLUDE_STMANAGER

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

class StreamManager {
    public:
	StreamManager();
	~StreamManager();
	cv::Mat getFrameAsMat(rs2_stream type);
	rs2::frame getFrameAsRS2Frame(rs2_stream type);
    private:
	rs2::pipeline pipe;
};

#endif
