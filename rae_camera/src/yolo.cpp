#include <chrono>
#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include "opencv2/opencv.hpp"
/*
The code is the same as for Tiny-yolo-V3, the only difference is the blob file.
The blob was compiled following this tutorial: https://github.com/TNTWEN/OpenVINO-YOLOV4
*/

static const std::vector<std::string> labelMap = {
    "person",        "bicycle",      "car",           "motorbike",     "aeroplane",   "bus",         "train",       "truck",        "boat",
    "traffic light", "fire hydrant", "stop sign",     "parking meter", "bench",       "bird",        "cat",         "dog",          "horse",
    "sheep",         "cow",          "elephant",      "bear",          "zebra",       "giraffe",     "backpack",    "umbrella",     "handbag",
    "tie",           "suitcase",     "frisbee",       "skis",          "snowboard",   "sports ball", "kite",        "baseball bat", "baseball glove",
    "skateboard",    "surfboard",    "tennis racket", "bottle",        "wine glass",  "cup",         "fork",        "knife",        "spoon",
    "bowl",          "banana",       "apple",         "sandwich",      "orange",      "broccoli",    "carrot",      "hot dog",      "pizza",
    "donut",         "cake",         "chair",         "sofa",          "pottedplant", "bed",         "diningtable", "toilet",       "tvmonitor",
    "laptop",        "mouse",        "remote",        "keyboard",      "cell phone",  "microwave",   "oven",        "toaster",      "sink",
    "refrigerator",  "book",         "clock",         "vase",          "scissors",    "teddy bear",  "hair drier",  "toothbrush"};

static std::atomic<bool> syncNN{false};

int main(int argc, char** argv) {
    using namespace std;
    using namespace std::chrono;
    std::string nnPath("/workspaces/rae_ws/yolov6n_416x416_openvino2022.1_vpux.blob");

    // If path to blob specified, use that
    if(argc > 1) {
        nnPath = std::string(argv[1]);
    }

    // Print which blob we are using
    printf("Using blob at path: %s\n", nnPath.c_str());

    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto spatialDetectionNetwork = pipeline.create<dai::node::YoloSpatialDetectionNetwork>();
    auto monoLeft = pipeline.create<dai::node::ColorCamera>();
    auto monoRight = pipeline.create<dai::node::ColorCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();

    auto xoutRgb = pipeline.create<dai::node::XLinkOut>();
    auto xoutNN = pipeline.create<dai::node::XLinkOut>();
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();
    auto nnNetworkOut = pipeline.create<dai::node::XLinkOut>();

    xoutRgb->setStreamName("rgb");
    xoutNN->setStreamName("detections");
    xoutDepth->setStreamName("depth");
    nnNetworkOut->setStreamName("nnNetwork");

    // Properties
    monoLeft->setPreviewSize(416, 416);
    monoLeft->setVideoSize(640, 480);
    monoRight->setVideoSize(640, 480);

    monoLeft->setInterleaved(false);

    monoLeft->setResolution(dai::ColorCameraProperties::SensorResolution::THE_800_P);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoRight->setResolution(dai::ColorCameraProperties::SensorResolution::THE_800_P);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

    // setting node configs
    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
    // Align depth map to the perspective of RGB camera, on which inference is done
    // stereo->setDepthAlign(dai::CameraBoardSocket::CAM_A);
    stereo->setOutputSize(monoLeft->getResolutionWidth(), monoLeft->getResolutionHeight());

    stereo->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_7x7);
    stereo->setLeftRightCheck(true);
    stereo->setExtendedDisparity(false);
    stereo->setSubpixel(false);
    auto config = stereo->initialConfig.get();
    config.postProcessing.speckleFilter.enable = false;
    config.postProcessing.speckleFilter.speckleRange = 50;
    config.postProcessing.temporalFilter.enable = true;
    config.postProcessing.spatialFilter.enable = true;
    config.postProcessing.spatialFilter.holeFillingRadius = 2;
    config.postProcessing.spatialFilter.numIterations = 1;
    config.postProcessing.thresholdFilter.minRange = 400;
    config.postProcessing.thresholdFilter.maxRange = 15000;
    config.postProcessing.decimationFilter.decimationFactor = 1;
    stereo->initialConfig.set(config);

    spatialDetectionNetwork->setBlobPath(nnPath);
    spatialDetectionNetwork->setConfidenceThreshold(0.5f);
    spatialDetectionNetwork->input.setBlocking(false);
    spatialDetectionNetwork->inputDepth.setBlocking(false);
    spatialDetectionNetwork->setBoundingBoxScaleFactor(0.5);
    spatialDetectionNetwork->setDepthLowerThreshold(100);
    spatialDetectionNetwork->setDepthUpperThreshold(5000);

    // yolo specific parameters
    spatialDetectionNetwork->setNumClasses(80);
    spatialDetectionNetwork->setCoordinateSize(4);
    spatialDetectionNetwork->setAnchors({10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319});
    spatialDetectionNetwork->setAnchorMasks({{"side26", {1, 2, 3}}, {"side13", {3, 4, 5}}});
    spatialDetectionNetwork->setIouThreshold(0.5f);
    spatialDetectionNetwork->setNumInferenceThreads(1);
    spatialDetectionNetwork->setNumShavesPerInferenceThread(1);

    // Linking
    monoLeft->video.link(stereo->left);
    monoRight->video.link(stereo->right);

    // manipL->out.link(stereo->left);
    // manipR->out.link(stereo->right);

    monoLeft->preview.link(spatialDetectionNetwork->input);
    if(syncNN) {
        spatialDetectionNetwork->passthrough.link(xoutRgb->input);
    } else {
        monoLeft->preview.link(xoutRgb->input);
    }

    spatialDetectionNetwork->out.link(xoutNN->input);

    stereo->depth.link(spatialDetectionNetwork->inputDepth);
    spatialDetectionNetwork->passthroughDepth.link(xoutDepth->input);
    spatialDetectionNetwork->outNetwork.link(nnNetworkOut->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Output queues will be used to get the rgb frames and nn data from the outputs defined above
    auto previewQueue = device.getOutputQueue("rgb", 4, false);
    auto detectionNNQueue = device.getOutputQueue("detections", 4, false);
    auto depthQueue = device.getOutputQueue("depth", 4, false);
    auto networkQueue = device.getOutputQueue("nnNetwork", 4, false);

    auto startTime = steady_clock::now();
    int counter = 0;
    float fps = 0;
    auto color = cv::Scalar(255, 255, 255);
    bool printOutputLayersOnce = true;
    while(true) {
        auto imgFrame = previewQueue->get<dai::ImgFrame>();
        auto inDet = detectionNNQueue->get<dai::SpatialImgDetections>();
        auto depth = depthQueue->get<dai::ImgFrame>();
        auto inNN = networkQueue->get<dai::NNData>();

        if(printOutputLayersOnce && inNN) {
            std::cout << "Output layer names: ";
            for(const auto& ten : inNN->getAllLayerNames()) {
                std::cout << ten << ", ";
            }
            std::cout << std::endl;
            printOutputLayersOnce = false;
        }
        cv::Mat frame;
        cv::Size s(imgFrame->getWidth(), imgFrame->getHeight());
        std::vector<cv::Mat> channels;
        // BGR
        channels.push_back(cv::Mat(s, CV_8UC1, imgFrame->getData().data() + s.area() * 0));
        channels.push_back(cv::Mat(s, CV_8UC1, imgFrame->getData().data() + s.area() * 1));
        channels.push_back(cv::Mat(s, CV_8UC1, imgFrame->getData().data() + s.area() * 2));
        cv::merge(channels, frame);
        cv::Mat depthFrame = cv::Mat(cv::Size(depth->getWidth(), depth->getHeight()), CV_16UC1, depth->getData().data());

        cv::Mat depthFrameColor;
        cv::normalize(depthFrame, depthFrameColor, 255, 0, cv::NORM_INF, CV_8UC1);
        cv::equalizeHist(depthFrameColor, depthFrameColor);
        cv::applyColorMap(depthFrameColor, depthFrameColor, cv::COLORMAP_JET);

        counter++;
        auto currentTime = steady_clock::now();
        auto elapsed = duration_cast<duration<float>>(currentTime - startTime);
        if(elapsed > seconds(1)) {
            fps = counter / elapsed.count();
            counter = 0;
            startTime = currentTime;
        }

        auto detections = inDet->detections;

        for(const auto& detection : detections) {
            auto roiData = detection.boundingBoxMapping;
            auto roi = roiData.roi;
            roi = roi.denormalize(depthFrameColor.cols, depthFrameColor.rows);
            auto topLeft = roi.topLeft();
            auto bottomRight = roi.bottomRight();
            auto xmin = (int)topLeft.x;
            auto ymin = (int)topLeft.y;
            auto xmax = (int)bottomRight.x;
            auto ymax = (int)bottomRight.y;
            cv::rectangle(depthFrameColor, cv::Rect(cv::Point(xmin, ymin), cv::Point(xmax, ymax)), color, cv::FONT_HERSHEY_SIMPLEX);

            int x1 = detection.xmin * frame.cols;
            int y1 = detection.ymin * frame.rows;
            int x2 = detection.xmax * frame.cols;
            int y2 = detection.ymax * frame.rows;

            uint32_t labelIndex = detection.label;
            std::string labelStr = to_string(labelIndex);
            if(labelIndex < labelMap.size()) {
                labelStr = labelMap[labelIndex];
            }
            cv::putText(frame, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
            std::stringstream confStr;
            confStr << std::fixed << std::setprecision(2) << detection.confidence * 100;
            cv::putText(frame, confStr.str(), cv::Point(x1 + 10, y1 + 35), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);

            std::stringstream depthX;
            depthX << "X: " << (int)detection.spatialCoordinates.x << " mm";
            cv::putText(frame, depthX.str(), cv::Point(x1 + 10, y1 + 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
            std::stringstream depthY;
            depthY << "Y: " << (int)detection.spatialCoordinates.y << " mm";
            cv::putText(frame, depthY.str(), cv::Point(x1 + 10, y1 + 65), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
            std::stringstream depthZ;
            depthZ << "Z: " << (int)detection.spatialCoordinates.z << " mm";
            cv::putText(frame, depthZ.str(), cv::Point(x1 + 10, y1 + 80), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);

            cv::rectangle(frame, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color, cv::FONT_HERSHEY_SIMPLEX);
        }

        std::stringstream fpsStr;
        fpsStr << std::fixed << std::setprecision(2) << fps;
        cv::putText(frame, fpsStr.str(), cv::Point(2, imgFrame->getHeight() - 4), cv::FONT_HERSHEY_TRIPLEX, 0.4, color);

        cv::imshow("depth", depthFrameColor);
        cv::imshow("rgb", frame);

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
    }
    return 0;
}