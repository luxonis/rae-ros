#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include "opencv2/opencv.hpp"
int main() {
    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto monoLeft = pipeline.create<dai::node::ColorCamera>();
    auto monoRight = pipeline.create<dai::node::ColorCamera>();
    auto xoutLeft = pipeline.create<dai::node::XLinkOut>();
    auto xoutRight = pipeline.create<dai::node::XLinkOut>();

    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");

    // Properties
    monoLeft->setResolution(dai::node::ColorCamera::Properties::SensorResolution::THE_800_P);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::CAM_D);
    monoLeft->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);

    // monoLeft->setInterleaved(false);
    monoLeft->initialControl.setMisc("stride-align", 1);
    monoLeft->initialControl.setMisc("scanline-align", 1);
    monoRight->setResolution(dai::node::ColorCamera::Properties::SensorResolution::THE_800_P);
    monoRight->setBoardSocket(dai::CameraBoardSocket::CAM_E);
    // monoRight->setInterleaved(false);
    monoRight->initialControl.setMisc("stride-align", 1);
    monoRight->initialControl.setMisc("scanline-align", 1);
    // Linking
    monoRight->video.link(xoutRight->input);
    monoLeft->video.link(xoutLeft->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);
    // Output queues will be used to get the grayscale frames from the outputs defined above
    auto qLeft = device.getOutputQueue("left", 30, false);
    auto qRight = device.getOutputQueue("right", 4, false);

    while(true) {
        // Instead of get (blocking), we use tryGet (non-blocking) which will return the available data or None otherwise
        auto inLeft = qLeft->get<dai::ImgFrame>();
        auto inRight = qRight->get<dai::ImgFrame>();
        cv::Mat leftFrame;
        cv::Size s(inLeft->getWidth(), inLeft->getHeight());
        std::vector<cv::Mat> channelsL;
        // // BGR
        channelsL.push_back(cv::Mat(s, CV_8UC1, inLeft->getData().data() + s.area() * 0));
        // channelsL.push_back(cv::Mat(s, CV_8UC1, inLeft->getData().data() + s.area() * 1));
        // channelsL.push_back(cv::Mat(s, CV_8UC1, inLeft->getData().data() + s.area() * 2));
        cv::merge(channelsL, leftFrame);
        cv::imshow("left", leftFrame);

        if(inRight) {
            cv::Mat rightFrame;
            cv::Size s(inRight->getWidth(), inRight->getHeight());
            std::vector<cv::Mat> channelsR;
            // BGR
            channelsR.push_back(cv::Mat(s, CV_8UC1, inRight->getData().data() + s.area() * 0));
            cv::merge(channelsR, rightFrame);
            cv::imshow("right", rightFrame);
        }

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
    }
    return 0;
}