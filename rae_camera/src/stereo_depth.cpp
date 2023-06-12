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

int main() {
    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto monoLeft = pipeline.create<dai::node::ColorCamera>();
    auto monoRight = pipeline.create<dai::node::ColorCamera>();
    auto depth = pipeline.create<dai::node::StereoDepth>();
    auto xout = pipeline.create<dai::node::XLinkOut>();

    xout->setStreamName("depth");

    // Properties
    monoLeft->setResolution(dai::node::ColorCamera::Properties::SensorResolution::THE_800_P);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoRight->setResolution(dai::node::ColorCamera::Properties::SensorResolution::THE_800_P);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

    // Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
    depth->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
    // Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
    depth->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_7x7);
    depth->setLeftRightCheck(lr_check);
    depth->setExtendedDisparity(extended_disparity);
    depth->setSubpixel(subpixel);
    auto config = depth->initialConfig.get();
    config.postProcessing.speckleFilter.enable = false;
    config.postProcessing.speckleFilter.speckleRange = 50;
    config.postProcessing.temporalFilter.enable = true;
    config.postProcessing.spatialFilter.enable = true;
    config.postProcessing.spatialFilter.holeFillingRadius = 2;
    config.postProcessing.spatialFilter.numIterations = 1;
    config.postProcessing.thresholdFilter.minRange = 400;
    config.postProcessing.thresholdFilter.maxRange = 15000;
    config.postProcessing.decimationFilter.decimationFactor = 1;
    depth->initialConfig.set(config);

    // Linking
    monoLeft->video.link(depth->left);
    monoRight->video.link(depth->right);
    depth->depth.link(xout->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Output queue will be used to get the disparity frames from the outputs defined above
    auto q = device.getOutputQueue("depth", 4, false);

    while(true) {
        auto inDepth = q->get<dai::ImgFrame>();
        // auto frame = inDepth->getData().data;
        // Normalization for better visualization
        // frame.convertTo(frame, CV_8UC1, 255 / depth->initialConfig.getMaxDisparity());
        auto frame = cv::Mat(cv::Size(inDepth->getWidth(), inDepth->getHeight()), CV_16UC1, inDepth->getData().data());
        cv::imshow("depth", frame);

        // Available color maps: https://docs.opencv.org/3.4/d3/d50/group__imgproc__colormap.html
        // cv::applyColorMap(frame, frame, cv::COLORMAP_JET);
        // cv::imshow("disparity_color", frame);

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
    }
    return 0;
}