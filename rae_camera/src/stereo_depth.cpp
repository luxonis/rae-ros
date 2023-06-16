#include <iostream>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

// Closer-in minimum depth, disparity range is doubled (from 95 to 190):
static std::atomic<bool> extended_disparity{false};
// Better accuracy for longer distance, fractional disparity 32-levels:
static std::atomic<bool> subpixel{false};
// Better handling for occlusions:
static std::atomic<bool> lr_check{true};

int main()
{
    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto xoutRgb = pipeline.create<dai::node::XLinkOut>();
    xoutRgb->setStreamName("rgb");
    auto monoLeftFront = pipeline.create<dai::node::ColorCamera>();
    auto monoRightFront = pipeline.create<dai::node::ColorCamera>();
    auto monoLeftBack = pipeline.create<dai::node::ColorCamera>();
    auto monoRightBack = pipeline.create<dai::node::ColorCamera>();
    auto depthFront = pipeline.create<dai::node::StereoDepth>();
    auto depthBack = pipeline.create<dai::node::StereoDepth>();
    auto xoutFront = pipeline.create<dai::node::XLinkOut>();
    auto xoutBack = pipeline.create<dai::node::XLinkOut>();


    xoutFront->setStreamName("depth_front");
    xoutBack->setStreamName("depth_back");

    // Properties
    monoLeftFront->setResolution(dai::node::ColorCamera::Properties::SensorResolution::THE_800_P);
    monoLeftFront->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoRightFront->setResolution(dai::node::ColorCamera::Properties::SensorResolution::THE_800_P);
    monoRightFront->setBoardSocket(dai::CameraBoardSocket::RIGHT);

    monoLeftBack->setResolution(dai::node::ColorCamera::Properties::SensorResolution::THE_800_P);
    monoLeftBack->setBoardSocket(dai::CameraBoardSocket::CAM_D);
    monoRightBack->setResolution(dai::node::ColorCamera::Properties::SensorResolution::THE_800_P);
    monoRightBack->setBoardSocket(dai::CameraBoardSocket::CAM_E);

    camRgb->setPreviewSize(416, 416);
    camRgb->setFps(15.0);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setInterleaved(false);
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);

    // Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
    depthFront->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
    depthFront->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_7x7);
    depthFront->setLeftRightCheck(lr_check);
    depthFront->setExtendedDisparity(extended_disparity);
    depthFront->setSubpixel(subpixel);
    auto config = depthFront->initialConfig.get();
    config.postProcessing.speckleFilter.enable = false;
    config.postProcessing.speckleFilter.speckleRange = 50;
    config.postProcessing.temporalFilter.enable = true;
    config.postProcessing.spatialFilter.enable = true;
    config.postProcessing.spatialFilter.holeFillingRadius = 2;
    config.postProcessing.spatialFilter.numIterations = 1;
    config.postProcessing.thresholdFilter.minRange = 400;
    config.postProcessing.thresholdFilter.maxRange = 15000;
    config.postProcessing.decimationFilter.decimationFactor = 1;
    depthFront->initialConfig.set(config);

    depthBack->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
    depthBack->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_7x7);
    depthBack->setLeftRightCheck(lr_check);
    depthBack->setExtendedDisparity(extended_disparity);
    depthBack->setSubpixel(subpixel);
    depthBack->initialConfig.set(config);

    // Linking
    monoLeftFront->video.link(depthFront->left);
    monoRightFront->video.link(depthFront->right);
    depthFront->depth.link(xoutFront->input);

    monoLeftBack->video.link(depthBack->left);
    monoRightBack->video.link(depthBack->right);
    depthBack->depth.link(xoutBack->input);

    camRgb->preview.link(xoutRgb->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Output queue will be used to get the disparity frames from the outputs defined above
    auto q_f = device.getOutputQueue("depth_front", 4, false);
    auto q_b = device.getOutputQueue("depth_back", 4, false);
    auto previewQueue = device.getOutputQueue("rgb", 4, false);

    while (true)
    {
        auto inDepthF = q_f->get<dai::ImgFrame>();

        auto frameF = cv::Mat(cv::Size(inDepthF->getWidth(), inDepthF->getHeight()), CV_16UC1, inDepthF->getData().data());
        cv::Mat depthFrameColorF;
        cv::normalize(frameF, depthFrameColorF, 255, 0, cv::NORM_INF, CV_8UC1);
        cv::equalizeHist(depthFrameColorF, depthFrameColorF);
        cv::applyColorMap(depthFrameColorF, depthFrameColorF, cv::COLORMAP_JET);

        auto inDepthB = q_b->get<dai::ImgFrame>();
        auto frameB = cv::Mat(cv::Size(inDepthB->getWidth(), inDepthB->getHeight()), CV_16UC1, inDepthB->getData().data());
        cv::Mat depthFrameColorB;
        cv::normalize(frameB, depthFrameColorB, 255, 0, cv::NORM_INF, CV_8UC1);
        cv::equalizeHist(depthFrameColorB, depthFrameColorB);
        cv::applyColorMap(depthFrameColorB, depthFrameColorB, cv::COLORMAP_JET);

        // auto imgFrame = previewQueue->get<dai::ImgFrame>();
        // cv::Mat frame;
        // cv::Size s(imgFrame->getWidth(), imgFrame->getHeight());
        // std::vector<cv::Mat> channels;
        // // BGR
        // channels.push_back(cv::Mat(s, CV_8UC1, imgFrame->getData().data() + s.area() * 0));
        // channels.push_back(cv::Mat(s, CV_8UC1, imgFrame->getData().data() + s.area() * 1));
        // channels.push_back(cv::Mat(s, CV_8UC1, imgFrame->getData().data() + s.area() * 2));
        // cv::merge(channels, frame);
        // cv::imshow("rgb", frame);

        cv::imshow("depth_f", depthFrameColorF);
        cv::imshow("depth_b", depthFrameColorB);
        int key = cv::waitKey(1);
        if (key == 'q' || key == 'Q')
        {
            return 0;
        }
    }
    return 0;
}