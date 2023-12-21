import os
import logging as log
import depthai as dai
import depthai_ros_py_bindings as dai_ros
import numpy as np
from .pipeline import rtabmap_pipeline
from ament_index_python import get_package_share_directory

ROBOTHUB_AVAILABLE=False
try:
    import robothub
    ROBOTHUB_AVAILABLE = True
except ImportError:
    log.error("RobotHub module is not available.")

class PerceptionSystem:
    """
    A class for managing camera functionalities in a robot, interfacing with both depthai and robothub libraries.
    It includes initialization and management of camera streams, publishing capabilities, and camera device control.

    Attributes:
        ros_context_manager (dai_ros.ROSContextManager): Manager for ROS context.
        dai_node (dai_ros.ROSNode): ROS node for depthai operations.
        device_mxid (str): The serial number of the depthai device.
        device_info (dai.DeviceInfo): Information about the depthai device.
        device (dai.Device): The depthai device instance.
        cal_handler: Calibration handler for the depthai device.
        pipeline: The pipeline for camera data processing.
        rh_stream_handles (dict): Handles for RobotHub video streams.
        ros_stream_handles (dict): Handles for ROS video streams.

    Methods:
        stop(): Closes the depthai device connection.
        add_rh_stream(stream_name): Adds a RobotHub stream with the given name.
        add_ros_img_stream(stream_name): Adds a ROS Image stream with the given name.
        add_ros_imu_stream(stream_name): Adds a ROS IMU stream with the given name.
        add_queue(name, callback): Adds a queue to the device for handling callbacks.
        start_pipeline(pipeline): Starts the camera pipeline and initializes ROS node and context.
        publish_rh(name, color_frame, timestamp, metadata): Publishes video data to RobotHub.
        publish_ros(name, msg): Publishes a message to a ROS topic.
        get_image(stream_name): Retrieves an image from the specified stream.
    """

    def __init__(self):
        """
        Initializes the Camera instance. 
        """
        self._pipeline = None
        self._ros_stream_handles = {}
        self._dai_node = None
        self._device = None
        self._ros_context_manager = dai_ros.ROSContextManager()
        self._queues = {}
        self._config_path = os.path.join(
            get_package_share_directory('robot_py'), 'config', 'example_config.yaml')
        if self.connect_to_device():
            log.info("Perception module initialized. Now please set up the pipeline.")
        else:
            log.error("Could not initialize perception module. You can try again by calling the connect_to_device() method.")

    def connect_to_device(self) -> bool:
        """
        Connects to the depthai device.
        """
        try:
            if not ROBOTHUB_AVAILABLE:
                self._device = dai.Device()
                self._cal_handler = self._device.readCalibration()
            else:
                self._device_mxid = robothub.DEVICES[0].oak["serialNumber"]
                self._device_info = dai.DeviceInfo(self._device_mxid)
                self._device = dai.Device(self._device_info)
                self._cal_handler = self._device.readCalibration()
                self._rh_stream_handles = {}
            return True
        except RuntimeError as e:
            log.error(f"Could not connect to the device. Reason: {e}")
            return False

    def start(self):
        """
        Starts the ROS context manager and spins the ROS node.
        """
        if self._pipeline is not None:
            self._ros_context_manager.add_node(self._dai_node)
            self._ros_context_manager.spin()
        else:
            log.error("Pipeline not set up. Please set up the pipeline.")

    def stop(self):
        """
        Closes the connection to the depthai device, ensuring a clean shutdown.
        """
        if self._pipeline is not None:
            self._ros_context_manager.shutdown()
        if self._device:
            self._device.close()
    def add_rh_stream(self, stream_name):
        """
        Adds a video stream to RobotHub with the specified name.

        Args:
            stream_name (str): The name of the stream to be added.
        """

        if ROBOTHUB_AVAILABLE:
            self._rh_stream_handles[stream_name] = robothub.STREAMS.create_video(
                self._device_mxid, stream_name, stream_name
            )
        else:
            log.error("RobotHub is not available. Cannot add RobotHub stream.")

    
    def add_ros_img_stream(self, stream_name, topic_name, frame_name, socket, width=-1, height=-1, convertFromBitStream=False, frame_type=dai.RawImgFrame.Type.BGR888i):
        """
        Adds a ROS video stream with the specified name and sets up the necessary configurations.

        Args:
            stream_name (str): The name of the ROS stream to be added.
        """
        log.info(
            f'Adding ROS stream {stream_name} with socket {socket} and frame name {frame_name}')
        self._ros_stream_handles[stream_name] = dai_ros.ImgStreamer(
            self._dai_node, self._cal_handler, socket, topic_name, frame_name, width, height)
        if convertFromBitStream:
            self._ros_stream_handles[stream_name].convertFromBitStream(
                frame_type)

    def add_ros_imu_stream(self, stream_name, frame_name):
        """
        Adds a ROS IMU stream with the specified name and sets up the necessary configurations.

        Args:
            stream_name (str): The name of the ROS stream to be added.
        """
        log.info(f'Adding ROS IMU stream {stream_name}')
        self._ros_stream_handles[stream_name] = dai_ros.ImuStreamer(
            self._dai_node, stream_name, frame_name, dai_ros.ImuSyncMethod.COPY, 0.0, 0.0, 0.0, 0.0, True, False, False)

    def add_queue(self, name, callback=None):
        """
        Adds a queue to the depthai device for processing callbacks.

        Args:
            name (str): The name of the queue.
            callback (callable): The callback function to be added to the queue.
        """
        log.info(f'Adding queue {name}')
        if self._device is None:
            log.error("Device is not connected. Cannot add queue.")
            return
        self._queues[name] = self._device.getOutputQueue(
            name, 1, False)
        if callback is not None:
            self._queues[name].addCallback(callback)

    def start_pipeline(self, pipeline):
        """
        Starts the camera pipeline, initializes the ROS node, and spins the ROS context manager.

        Args:
            pipeline: The pipeline configuration for the camera.
        """
        log.info("Starting pipeline...")
        if (self._device is None):
            log.error("Device is not connected. Cannot start pipeline.")
            return
        self._device.startPipeline(pipeline)
        self._pipeline = pipeline
        self._ros_context_manager = dai_ros.ROSContextManager()
        self._ros_context_manager.init([""], "single_threaded")
        self.opts = dai_ros.ROSNodeOptions()
        self._dai_node = dai_ros.ROSNode("dai", self.opts)
        log.info("Pipeline started.")

    def publish_rh(self, name, color_frame, timestamp, metadata):
        """
        Publishes video data to a RobotHub stream.

        Args:
            name (str): The name of the RobotHub stream.
            color_frame: The color frame data to be published.
            timestamp: The timestamp associated with the frame.
            metadata: Additional metadata for the frame.
        """
        if ROBOTHUB_AVAILABLE:
            self._rh_stream_handles[name].publish_video_data(
                bytes(color_frame), timestamp, metadata)
        else:
            log.error("RobotHub is not available. Cannot publish to RobotHub.")

    def publish_ros(self, name, msg):
        """
        Publishes a message to a ROS topic.

        Args:
            name (str): The name of the ROS topic.
            msg: The message to be published.
        """
        self._ros_stream_handles[name].publish(name, msg)

    def get_image(self, stream_name) -> np.ndarray:
        """
        Retrieves an image from the specified stream.

        Args:
            stream_name (str): The name of the stream to retrieve the image from.

        Returns:
            Image: The image retrieved from the stream.
        """
        return self._queues[stream_name].get().getCvFrame()

    def stream_name_to_socket(self, stream_name):
        if stream_name == 'stream_front':
            return dai.CameraBoardSocket.CAM_C
        elif stream_name == 'stream_back':
            return dai.CameraBoardSocket.CAM_D

    def setup_perception_rtabmap(self):
        name = '/rae'
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
        left.setResolution(
            dai.ColorCameraProperties.SensorResolution.THE_800_P)
        left.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        left.setFps(30)
        left.setVideoSize(640, 400)
        left.setPreviewSize(416, 416)
        left.setInterleaved(False)

        right = pipeline.create(dai.node.ColorCamera)
        right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        right.setResolution(
            dai.ColorCameraProperties.SensorResolution.THE_800_P)
        left.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
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

        self.start_pipeline(pipeline)
        calHandler = self._device.readCalibration()
        self.opts_rtabmap = dai_ros.ROSNodeOptions(False, "/rtabmap", self._config_path,
                                                   {"odom": "/diff_controller/odom",
                                                    "rgb/image": name+"/right/image_rect",
                                                    "rgb/camera_info": name+"/right/camera_info",
                                                    "depth/image": name+"/stereo/image_raw"})
        self.opts = dai_ros.ROSNodeOptions(
            False, "/dai", self._config_path, {"t": "t"})
        self._dai_node = dai_ros.ROSNode("dai", self.opts)

        self._ros_stream_handles['imu'] = dai_ros.ImuStreamer(
            self._dai_node, "/rae/imu/data", "rae_imu_frame", dai_ros.ImuSyncMethod.COPY, 0.0, 0.0, 0.0, 0.0, True, False, False)
        self._ros_stream_handles['left'] = dai_ros.ImgStreamer(
            self._dai_node, calHandler, dai.CameraBoardSocket.CAM_B, "/rae/left/image_rect", "rae_left_camera_optical_frame", 640, 400)
        self._ros_stream_handles['right'] = dai_ros.ImgStreamer(
            self._dai_node, calHandler, dai.CameraBoardSocket.CAM_C, "/rae/right/image_raw", "rae_right_camera_optical_frame", 640, 400)
        self._ros_stream_handles['stereo'] = dai_ros.ImgStreamer(
            self._dai_node, calHandler, dai.CameraBoardSocket.CAM_C, "/rae/stereo/image_raw", "rae_right_camera_optical_frame", 640, 400)
        self._device.getOutputQueue(
            "imu", 8, False).addCallback(self.publish_ros)
        self._device.getOutputQueue(
            "stereo", 8, False).addCallback(self.publish_ros)
        self._device.getOutputQueue(
            "left", 8, False).addCallback(self.publish_ros)
        self._device.getOutputQueue(
            "right", 8, False).addCallback(self.publish_ros)
        self.scan_front_opts = dai_ros.ROSNodeOptions(False, "/laserscan_kinect_front", self._config_path,
                                                      {"laserscan_kinect:__node": "laserscan_kinect_front",
                                                       "/image": name+"/stereo/image_raw",
                                                       "/camera_info": name+"/stereo/camera_info",
                                                       "/scan": name+"/scan_front",
                                                       "/debug_image": name+"/debug_image_front",
                                                       "/debug_image/compressed": name+"/debug_image_front/compressed"})
        self.laserscan_front = dai_ros.LaserScanKinectNode(
            self.scan_front_opts)

        self.opts_rectify = dai_ros.ROSNodeOptions(False, "/rectify", self._config_path, {
                                                   "image": "/rae/right/image_raw", "camera_info": "/rae/right/camera_info", "image_rect": "/rae/right/image_rect"})

        self.rectify = dai_ros.ImageProcRectifyNode(self.opts_rectify)
        # self._ros_context_manager.add_node(self._dai_node)
        self._ros_context_manager.add_node(self.laserscan_front)
        self._ros_context_manager.add_node(self.rectify)
