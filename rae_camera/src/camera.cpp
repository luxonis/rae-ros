#include <cstdio>
#include <functional>
#include <iostream>
#include <tuple>

#include "camera_info_manager/camera_info_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"

dai::Pipeline createPipeline(bool enable_rgb, bool enable_depth)
{
    dai::Pipeline pipeline;
    if (enable_rgb)
    {
        auto camRgb = pipeline.create<dai::node::ColorCamera>();
        camRgb->setResolution(dai::node::ColorCamera::Properties::SensorResolution::THE_1080_P);
        auto xoutRgb = pipeline.create<dai::node::XLinkOut>();
        camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
        camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
        camRgb->setInterleaved(false);
        camRgb->setVideoSize(1280,800);
        camRgb->initialControl.setMisc("stride-align", 1);
        camRgb->initialControl.setMisc("scanline-align", 1);
        xoutRgb->setStreamName("rgb");
        camRgb->video.link(xoutRgb->input);
    }
    if (enable_depth)
    {
    auto leftFront = pipeline.create<dai::node::ColorCamera>();
    auto rightFront = pipeline.create<dai::node::ColorCamera>();
    auto leftBack = pipeline.create<dai::node::ColorCamera>();
    auto rightBack = pipeline.create<dai::node::ColorCamera>();
    auto depthFront = pipeline.create<dai::node::StereoDepth>();
    auto depthBack = pipeline.create<dai::node::StereoDepth>();
    auto xoutFront = pipeline.create<dai::node::XLinkOut>();
    auto xoutBack = pipeline.create<dai::node::XLinkOut>();
    auto xoutRightFront = pipeline.create<dai::node::XLinkOut>();

    xoutFront->setStreamName("depth_front");
    xoutRightFront->setStreamName("right_front");
    xoutBack->setStreamName("depth_back");
    rightFront->initialControl.setMisc("stride-align", 1);
    rightFront->initialControl.setMisc("scanline-align", 1);

    // Properties
    leftFront->setResolution(dai::node::ColorCamera::Properties::SensorResolution::THE_800_P);
    leftFront->setBoardSocket(dai::CameraBoardSocket::LEFT);
    rightFront->setResolution(dai::node::ColorCamera::Properties::SensorResolution::THE_800_P);
    rightFront->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    leftFront->setVideoSize(640, 400);
    rightFront->setVideoSize(640,400);
    // leftFront->setFps(15.0);
    // rightFront->setFps(15.0);

    leftBack->setResolution(dai::node::ColorCamera::Properties::SensorResolution::THE_800_P);
    leftBack->setBoardSocket(dai::CameraBoardSocket::CAM_D);
    rightBack->setResolution(dai::node::ColorCamera::Properties::SensorResolution::THE_800_P);
    rightBack->setBoardSocket(dai::CameraBoardSocket::CAM_E);
    leftBack->setVideoSize(640,400);
    rightBack->setVideoSize(640, 400);
    // leftBack->setFps(15.0);
    // rightBack->setFps(15.0);

    // Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
    depthFront->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
    depthFront->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_7x7);
    depthFront->setLeftRightCheck(10);
    depthFront->setExtendedDisparity(false);
    depthFront->setSubpixel(true);
    auto config = depthFront->initialConfig.get();
    config.postProcessing.speckleFilter.enable = false;
    config.postProcessing.speckleFilter.speckleRange = 50;
    config.postProcessing.temporalFilter.enable = false;
    config.postProcessing.spatialFilter.enable = false;
    config.postProcessing.spatialFilter.holeFillingRadius = 2;
    config.postProcessing.spatialFilter.numIterations = 1;
    config.postProcessing.thresholdFilter.minRange = 400;
    config.postProcessing.thresholdFilter.maxRange = 15000;
    config.postProcessing.decimationFilter.decimationFactor = 1;
    depthFront->initialConfig.set(config);

    depthBack->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
    depthBack->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_7x7);
    depthBack->setLeftRightCheck(10);
    depthBack->setExtendedDisparity(false);
    depthBack->setSubpixel(true);
    depthBack->initialConfig.set(config);

    // Linking
    leftFront->video.link(depthFront->left);
    rightFront->video.link(depthFront->right);
    rightFront->video.link(xoutRightFront->input);
    depthFront->depth.link(xoutFront->input);

    leftBack->video.link(depthBack->left);
    rightBack->video.link(depthBack->right);
    depthBack->depth.link(xoutBack->input);
    }
    return pipeline;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("rae_camera");
    bool enable_rgb = node->declare_parameter<bool>("enable_rgb", true);
    bool enable_depth = node->declare_parameter<bool>("enable_depth", true);

    dai::Pipeline pipeline;
    pipeline = createPipeline(enable_rgb, enable_depth);

    std::shared_ptr<dai::Device> device;

    device = std::make_shared<dai::Device>(pipeline);

    auto calibrationHandler = device->readCalibration();

    // ROS part

    std::string tfPrefix = "rae";

    int width = 640;
    int height = 400;
    std::shared_ptr<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame>> rgbPublish;
    std::shared_ptr<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame>> depthFrontPublish;
    std::shared_ptr<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame>> rightFrontPublish;
    std::shared_ptr<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame>> depthBackPublish;
    dai::rosBridge::ImageConverter stereoFrontConverter(tfPrefix + "_right_front_camera_optical_frame", true);
    dai::rosBridge::ImageConverter stereoBackConverter(tfPrefix + "_right_back_camera_optical_frame", true);
    dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", false);

    if (enable_depth)
    {
        auto depthFrontCameraInfo = stereoFrontConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, width, height);

        auto stereoFrontQueue = device->getOutputQueue("depth_front", 8, false);
        depthFrontPublish = std::make_shared<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame>>(
            stereoFrontQueue,
            node,
            std::string("/rae/stereo_front/image_raw"),
            std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                      &stereoFrontConverter, // since the converter has the same frame name
                                       // and image type is also same we can reuse it
                      std::placeholders::_1,
                      std::placeholders::_2),
            30,
            depthFrontCameraInfo,
            "/rae/stereo_front");
        depthFrontPublish->addPublisherCallback();

        auto rightFrontQueue = device->getOutputQueue("right_front", 8, false);
        rightFrontPublish = std::make_shared<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame>>(
            rightFrontQueue,
            node,
            std::string("/rae/right_front/image_raw"),
            std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                      &stereoFrontConverter, // since the converter has the same frame name
                                       // and image type is also same we can reuse it
                      std::placeholders::_1,
                      std::placeholders::_2),
            30,
            depthFrontCameraInfo,
            "/rae/right_front");
        rightFrontPublish->addPublisherCallback();


        auto depthBackCameraInfo = stereoBackConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_E, width, height);

        auto stereoBackQueue = device->getOutputQueue("depth_back", 8, false);
        depthBackPublish = std::make_shared<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame>>(
            stereoBackQueue,
            node,
            std::string("/rae/stereo_back/image_raw"),
            std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                      &stereoBackConverter, // since the converter has the same frame name
                                       // and image type is also same we can reuse it
                      std::placeholders::_1,
                      std::placeholders::_2),
            30,
            depthBackCameraInfo,
            "/rae/stereo_back");
        depthBackPublish->addPublisherCallback();

    }
    if (enable_rgb)
    {
        sensor_msgs::msg::CameraInfo rgbCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RIGHT, width, height);;
        auto imgQueue = device->getOutputQueue("rgb", 30, false);
        rgbPublish = std::make_shared<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame>>(
            imgQueue,
            node,
            std::string("color/image"),
            std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rgbConverter, std::placeholders::_1, std::placeholders::_2),
            30,
            rgbCameraInfo,
            "color");
        rgbPublish->addPublisherCallback();
    }
    RCLCPP_INFO(node->get_logger(), "Camera set up, starting publishing.");
    auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    exec->add_node(node);
    exec->spin();
    RCLCPP_INFO(node->get_logger(), "Shutting down.");
    rclcpp::shutdown();
    return 0;
}
