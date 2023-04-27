#include <cstdio>
#include <functional>
#include <iostream>
#include <tuple>

#include "camera_info_manager/camera_info_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "stereo_msgs/msg/disparity_image.hpp"
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

std::vector<std::string> usbStrings = {"UNKNOWN", "LOW", "FULL", "HIGH", "SUPER", "SUPER_PLUS"};

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
        xoutRgb->setStreamName("rgb");
        camRgb->video.link(xoutRgb->input);
    }
    if (enable_depth)
    {
        auto left = pipeline.create<dai::node::ColorCamera>();
        auto right = pipeline.create<dai::node::ColorCamera>();
        auto stereo = pipeline.create<dai::node::StereoDepth>();
        auto xoutDepth = pipeline.create<dai::node::XLinkOut>();

        // MonoCamera
        left->setResolution(dai::node::ColorCamera::Properties::SensorResolution::THE_800_P);
        left->setBoardSocket(dai::CameraBoardSocket::LEFT);
        left->setFps(30.0);
        right->setResolution(dai::node::ColorCamera::Properties::SensorResolution::THE_800_P);
        right->setBoardSocket(dai::CameraBoardSocket::RIGHT);
        right->setFps(30.0);

        // StereoDepth
        stereo->initialConfig.setConfidenceThreshold(245);    // Known to be best
        stereo->setRectifyEdgeFillColor(0);                   // black, to better see the cutout
        stereo->initialConfig.setLeftRightCheckThreshold(10); // Known to be best
        stereo->setLeftRightCheck(true);
        stereo->setSubpixel(true);
        // XLinkOut
        xoutDepth->setStreamName("depth");

        // Link plugins CAM -> STEREO -> XLINK
        left->video.link(stereo->left);
        right->video.link(stereo->right);

        stereo->depth.link(xoutDepth->input);
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

    auto boardName = calibrationHandler.getEepromData().boardName;
    std::string tfPrefix = "rae";


    int width = 1280;
    int height = 800;
    if (enable_depth)
    {
        dai::rosBridge::ImageConverter rightconverter(tfPrefix + "_right_camera_optical_frame", true);
        auto rightCameraInfo = rightconverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RIGHT, width, height);
        auto depthCameraInfo = rightCameraInfo;
        auto depthconverter = rightconverter;

        auto stereoQueue = device->getOutputQueue("depth", 30, false);
        dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> depthPublish(
            stereoQueue,
            node,
            std::string("stereo/depth"),
            std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                      &depthconverter, // since the converter has the same frame name
                                       // and image type is also same we can reuse it
                      std::placeholders::_1,
                      std::placeholders::_2),
            30,
            depthCameraInfo,
            "stereo");
        depthPublish.addPublisherCallback();
    }
    if (enable_rgb)
    {
        dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", false);
        sensor_msgs::msg::CameraInfo rgbCameraInfo;
        auto imgQueue = device->getOutputQueue("rgb", 30, false);
        dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rgbPublish(
            imgQueue,
            node,
            std::string("color/image"),
            std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rgbConverter, std::placeholders::_1, std::placeholders::_2),
            30,
            rgbCameraInfo,
            "color");
        rgbPublish.addPublisherCallback();
    }
    rclcpp::spin(node);
    return 0;
}
