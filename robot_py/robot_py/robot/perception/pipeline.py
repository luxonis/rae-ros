from pathlib import Path

import depthai as dai

BLOB_PATH = Path("/app/yolov6n_416x416_openvino2022.1_vpux.blob")


def build_pipeline(front_socket: dai.CameraBoardSocket, front_stream_name, rear_socket, rear_stream_name)-> dai.Pipeline:
    pipeline = dai.Pipeline()
    imu = pipeline.create(dai.node.IMU)
    imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 400)
    imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 400)
    imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, 400)
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)
    xout_imu = pipeline.create(dai.node.XLinkOut)
    xout_imu.setStreamName("imu")
    imu.out.link(xout_imu.input)

    def add_side(socket: dai.CameraBoardSocket, stream_name):
        rgb = pipeline.create(dai.node.ColorCamera)
        rgb.setBoardSocket(socket)
        rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
        rgb.setInterleaved(False)
        rgb.setPreviewSize(416, 416)
        rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        rgb.setFps(8)  # Limitation of the current preview source at 416x416 resolution

        h264_encoder = pipeline.create(dai.node.VideoEncoder)
        h264_encoder.setDefaultProfilePreset(8, dai.VideoEncoderProperties.Profile.H264_MAIN)
        h264_encoder.setQuality(50)
        h264_encoder.setKeyframeFrequency(30)
        h264_encoder.setBitrateKbps(1800)
        h264_encoder.input.setQueueSize(1)
        rgb.video.link(h264_encoder.input)

        xout_color_h264 = pipeline.create(dai.node.XLinkOut)
        xout_color_h264.setStreamName(f"{stream_name}")
        xout_color_h264.input.setBlocking(False)
        h264_encoder.bitstream.link(xout_color_h264.input)
        
        mjpeg_encoder = pipeline.create(dai.node.VideoEncoder)
        mjpeg_encoder.setDefaultProfilePreset(8, dai.VideoEncoderProperties.Profile.MJPEG)
        mjpeg_encoder.setQuality(50)
        mjpeg_encoder.setKeyframeFrequency(30)
        mjpeg_encoder.setBitrateKbps(1800)
        mjpeg_encoder.input.setQueueSize(1)
        rgb.video.link(mjpeg_encoder.input)

        mjpeg_xout_color = pipeline.create(dai.node.XLinkOut)
        mjpeg_xout_color.setStreamName(f"{stream_name}_mjpeg")
        mjpeg_xout_color.input.setBlocking(False)
        mjpeg_encoder.bitstream.link(mjpeg_xout_color.input)

        detectionNetwork = pipeline.createYoloDetectionNetwork()
        detectionNetwork.setConfidenceThreshold(0.6)
        detectionNetwork.setNumClasses(80)
        detectionNetwork.setCoordinateSize(4)
        detectionNetwork.setIouThreshold(0.5)
        detectionNetwork.setBlobPath(BLOB_PATH)
        detectionNetwork.setNumInferenceThreads(8)
        detectionNetwork.input.setBlocking(False)
        detectionNetwork.input.setQueueSize(1)
        rgb.preview.link(detectionNetwork.input)

        xout_nn = pipeline.create(dai.node.XLinkOut)
        xout_nn.input.setBlocking(False)
        xout_nn.setStreamName(f"{stream_name}_nn")
        detectionNetwork.out.link(xout_nn.input)

    add_side(front_socket, front_stream_name)
    add_side(rear_socket, rear_stream_name)
    return pipeline

def rtabmap_pipeline():
        pipeline = dai.Pipeline()

        imu = pipeline.create(dai.node.IMU)
        imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 400)
        imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 400)
        imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, 400)
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)
        xout_imu = pipeline.create(dai.node.XLinkOut)
        xout_imu.setStreamName("imu")
        imu.out.link(xout_imu.input)

        left = pipeline.create(dai.node.ColorCamera)
        left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        left.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
        left.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        left.setFps(30)
        left.setVideoSize(640, 400)
        # left.setPreviewSize(416, 416)
        left.setInterleaved(False)

        right = pipeline.create(dai.node.ColorCamera)
        right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        right.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
        right.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        right.setFps(30)
        right.setVideoSize(640, 400)
        right.setInterleaved(False)
        right.initialControl.setMisc('stride-align', 1)
        right.initialControl.setMisc('scanline-align', 1)
        stereo = pipeline.create(dai.node.StereoDepth)
        left.video.link(stereo.left)
        right.video.link(stereo.right)

        xout_stereo = pipeline.create(dai.node.XLinkOut)
        xout_stereo.setStreamName("stereo")
        xout_stereo.input.setBlocking(False)
        stereo.depth.link(xout_stereo.input)

        xout_left = pipeline.create(dai.node.XLinkOut)
        xout_left.setStreamName("left")
        xout_left.input.setBlocking(False)
        stereo.rectifiedLeft.link(xout_left.input)
        xout_right = pipeline.create(dai.node.XLinkOut)
        xout_right.setStreamName("right")
        xout_right.input.setBlocking(False)
        right.video.link(xout_right.input)
        return pipeline