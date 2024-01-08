import os
import logging as log
import depthai as dai
import depthai_ros_py_bindings as dai_ros
import numpy as np
from .pipeline import rtabmap_pipeline, sai_pipeline
from ament_index_python import get_package_share_directory

ROBOTHUB_AVAILABLE = False
try:
    import robothub
    ROBOTHUB_AVAILABLE = True
except ImportError:
    log.error("RobotHub module is not available.")


class PerceptionSystem:
    """
    A class for managing camera functionalities in a robot, interfacing with both depthai and robothub libraries.

    It includes initialization and management of camera streams, publishing capabilities, and camera device control.

    Attributes
    ----------
        ros_context_manager (dai_ros.ROSContextManager): Manager for ROS context.
        dai_node (dai_ros.ROSNode): ROS node for depthai operations.
        device_mxid (str): The serial number of the depthai device.
        device_info (dai.DeviceInfo): Information about the depthai device.
        device (dai.Device): The depthai device instance.
        cal_handler: Calibration handler for the depthai device.
        pipeline: The pipeline for camera data processing.
        rh_stream_handles (dict): Handles for RobotHub video streams.
        ros_stream_handles (dict): Handles for ROS video streams.

    Methods
    -------
        stop(): Closes the depthai device connection.
        add_rh_stream(stream_name): Adds a RobotHub stream with the given name.
        add_ros_img_stream(stream_name): Adds a ROS Image stream with the given name.
        add_ros_imu_stream(stream_name): Adds a ROS IMU stream with the given name.
        add_queue(name, callback): Adds a queue to the device for handling callbacks.
        add_composable_node(package_name, plugin_name, options): Adds a composable node to the ROS context manager.
        start_pipeline(pipeline): Starts the camera pipeline and initializes ROS node and context.
        publish_rh(name, color_frame, timestamp, metadata): Publishes video data to RobotHub.
        publish_ros(name, msg): Publishes a message to a ROS topic.
        get_image(stream_name): Retrieves an image from the specified stream.

    """

    def __init__(self, namespace=''):
        """
        Initialize the Camera instance.

        Connect to the depthai device and initializes the ROS context manager.

        Args:
        ----
            namespace (str, optional): The namespace for the ROS nodes. Defaults to ''.

        """
        self._namespace = namespace
        self._pipeline = None
        self._ros_stream_handles = {}
        self._dai_node = None
        self._device = None
        self._ros_context_manager = None
        self._executor_type = "single_threaded"
        self._queues = {}
        self._config_path = os.path.join(
            get_package_share_directory('robot_py'), 'config', 'example_config.yaml')
        if self.connect_to_device():
            log.info(
                "Perception module initialized. Now please set up the pipeline.")
        else:
            log.error(
                "Could not initialize perception module. You can try again by calling the connect_to_device() method.")

    def __del__(self):
        self.stop()

    def set_executor_type(self, executor_type):
        self._executor_type = executor_type

    def connect_to_device(self) -> bool:
        """Connect to the depthai device and initialize the calibration handler."""
        log.info("Connecting to the device...")
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
        """Start the ROS context manager and spins the ROS node."""
        if self._pipeline is not None:
            self._ros_context_manager.add_node(self._dai_node)
            self._ros_context_manager.spin()
        else:
            log.error("Pipeline not set up. Please set up the pipeline.")

    def stop(self):
        """Close the connection to the depthai device, ensuring a clean shutdown."""
        if self._pipeline is not None:
            self._ros_context_manager.shutdown()
        if self._device:
            self._device.close()

    def add_rh_stream(self, stream_name):
        """
        Add a video stream to RobotHub with the specified name.

        Args:
        ----
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
        Add a ROS video stream with the specified name and sets up the necessary configurations.

        Args:
        ----
            stream_name (str): The name of the ROS stream to be added.
            topic_name (str): The name of the ROS topic to be published to.
            frame_name (str): The name of the ROS frame.
            socket (dai.CameraBoardSocket): The socket to be used for the stream.
            width (int, optional): The width of the stream. Defaults to -1.
            height (int, optional): The height of the stream. Defaults to -1.
            convertFromBitStream (bool, optional): Whether to convert the stream from a bitstream. Defaults to False.
            frame_type (dai.RawImgFrame.Type, optional): The type of the frame. Defaults to dai.RawImgFrame.Type.BGR888i.

        """
        log.info(
            f'Adding ROS stream {stream_name} with socket {socket} and frame name {frame_name}')
        self._ros_stream_handles[stream_name] = dai_ros.ImgStreamer(
            self._dai_node, self._cal_handler, socket, topic_name, frame_name, width, height)
        if convertFromBitStream:
            self._ros_stream_handles[stream_name].convertFromBitStream(
                frame_type)

    def add_ros_imu_stream(self, stream_name, topic_name, frame_name):
        """
        Add a ROS IMU stream with the specified name and sets up the necessary configurations.

        Args:
        ----
            stream_name (str): The name of the ROS stream to be added.
            topic_name (str): The name of the ROS topic to be published to.
            frame_name (str): The name of the ROS frame.

        """
        log.info(f'Adding ROS IMU stream {stream_name}')
        self._ros_stream_handles[stream_name] = dai_ros.ImuStreamer(
            self._dai_node, topic_name, frame_name, dai_ros.ImuSyncMethod.COPY, 0.0, 0.0, 0.0, 0.0, True, False, False)

    def add_ros_feature_stream(self, stream_name, topic_name, frame_name):
        """
        Add a ROS feature stream with the specified name and sets up the necessary configurations.

        Args:
        ----
            stream_name (str): The name of the ROS stream to be added.
            topic_name (str): The name of the ROS topic to be published to.
            frame_name (str): The name of the ROS frame.

        """
        log.info(f'Adding ROS feature stream {stream_name}')
        self._ros_stream_handles[stream_name] = dai_ros.TrackedFeaturesStreamer(
            self._dai_node, topic_name, frame_name)

    def add_queue(self, name, callback=None):
        """
        Add a queue to the depthai device for processing callbacks.

        Args:
        ----
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

    def add_composable_node(self, package_name, plugin_name, options=dai_ros.ROSNodeOptions()):
        """
        Add a composable node to the ROS context manager.

        Args:
        ----
            package_name (str): The name of the ROS package.
            plugin_name (str): The name of the ROS plugin.
            options (dai_ros.ROSNodeOptions, optional): The options for the ROS node. Defaults to dai_ros.ROSNodeOptions().

        """
        self._ros_context_manager.add_composable_node(
            package_name, plugin_name, options)

    def start_pipeline(self, pipeline):
        """
        Start the camera pipeline, initializes the ROS node, and spins the ROS context manager.

        Args:
        ----
            pipeline: The pipeline configuration for the camera.

        """
        log.info("Starting pipeline...")
        if (self._device is None):
            log.error("Device is not connected. Cannot start pipeline.")
            return
        self._device.startPipeline(pipeline)
        self._pipeline = pipeline
        self._ros_context_manager = dai_ros.ROSContextManager()
        self._ros_context_manager.init([""], self._executor_type)
        self.opts = dai_ros.ROSNodeOptions("dai", self._namespace)
        self._dai_node = dai_ros.ROSNode("dai", self.opts)
        log.info("Pipeline started.")

    def publish_rh(self, name, color_frame, timestamp, metadata):
        """
        Publish video data to a RobotHub stream.

        Args:
        ----
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
        Publish a message to a ROS topic.

        Args:
        ----
            name (str): The name of the ROS topic.
            msg: The message to be published.

        """
        self._ros_stream_handles[name].publish(name, msg)

    def get_image(self, stream_name) -> np.ndarray:
        """
        Retrieve an image from the specified stream.

        Args:
        ----
            stream_name (str): The name of the stream to retrieve the image from.

        Returns
        -------
            Image: The image retrieved from the stream in OpenCV format.

        """
        return self._queues[stream_name].get().getCvFrame()

    def stream_name_to_socket(self, stream_name):
        if stream_name == 'stream_front':
            return dai.CameraBoardSocket.CAM_C
        elif stream_name == 'stream_back':
            return dai.CameraBoardSocket.CAM_D

    def setup_sai_slam(self):
        self.set_executor_type("multi_threaded")
        self.start_pipeline(sai_pipeline())
        self.add_ros_imu_stream("imu", "imu/data", "rae_imu_frame")
        self.add_ros_feature_stream(
            "trackedFeaturesRight", "right_rect_feature_tracker/tracked_features", "rae_right_camera_optical_frame")
        self.add_ros_img_stream(
            "left", "left/image_rect", "rae_left_camera_optical_frame", dai.CameraBoardSocket.CAM_B, 640, 400)
        self.add_ros_img_stream("right", "right/image_rect",
                                "rae_right_camera_optical_frame", dai.CameraBoardSocket.CAM_C, 640, 400)
        self.add_ros_img_stream("stereo", "stereo/image_raw",
                                "rae_right_camera_optical_frame", dai.CameraBoardSocket.CAM_C, 640, 400)
        self.add_queue("imu", self.publish_ros)
        self.add_queue("left", self.publish_ros)
        self.add_queue("right", self.publish_ros)
        self.add_queue("stereo", self.publish_ros)
        self.add_queue("trackedFeaturesRight", self.publish_ros)

        config_path = os.path.join(get_package_share_directory(
            'robot_py'), 'config', 'example_config.yaml')
        self.opts = dai_ros.ROSNodeOptions("spetacularAI", self._namespace, config_path,
                                           {'input/imu': 'imu/data',
                                            '/input/cam0/image_rect': 'right/image_rect',
                                            '/input/cam1/image_rect': 'left/image_rect',
                                            '/input/cam0/camera_info': 'right/camera_info',
                                            '/input/cam1/camera_info': 'left/camera_info',
                                            '/input/depth/image': 'stereo/image_raw',
                                            '/input/cam0/features': 'right_rect_feature_tracker/tracked_features'})

        self.add_composable_node('spectacularai_ros2',
                                 'spectacularAI::ros2::Node', self.opts)

    def setup_rtabmap(self):
        self.set_executor_type("multi_threaded")

        self.start_pipeline(rtabmap_pipeline())

        self.add_ros_imu_stream("imu", "imu/data", "rae_imu_frame")
        self.add_ros_img_stream(
            "right", "right/image_raw", "rae_right_camera_optical_frame", dai.CameraBoardSocket.CAM_C, 640, 400)

        self.add_ros_img_stream("stereo", "stereo/image_raw",
                                "rae_right_camera_optical_frame", dai.CameraBoardSocket.CAM_C, 640, 400)

        self.add_queue("imu", self.publish_ros)
        self.add_queue("left", self.publish_ros)
        self.add_queue("right", self.publish_ros)
        self.add_queue("stereo", self.publish_ros)
        self.scan_front_opts = dai_ros.ROSNodeOptions("laserscan_kinect_front", self._namespace, self._config_path,
                                                      {"/image": "stereo/image_raw",
                                                       "/camera_info": "stereo/camera_info",
                                                       "/scan": "scan_front",
                                                       "/debug_image": "debug_image_front",
                                                       "/debug_image/compressed": "debug_image_front/compressed"})

        self.add_composable_node(
            'laserscan_kinect', 'laserscan_kinect::LaserScanKinectNode', self.scan_front_opts)

        self.opts_rectify = dai_ros.ROSNodeOptions("RectifyNode", self._namespace, self._config_path, {
                                                   "image": "right/image_raw", "camera_info": "right/camera_info", "image_rect": "right/image_rect"})

        self.add_composable_node(
            'depthai_filters', 'depthai_filters::Rectify', self.opts_rectify)
        self.opts_rtabmap = dai_ros.ROSNodeOptions("rtabmap", self._namespace, self._config_path,
                                                   {"odom": "diff_controller/odom",
                                                    "rgb/image": "right/image_rect",
                                                    "rgb/camera_info": "right/camera_info",
                                                    "depth/image": "stereo/image_raw"})
        self.add_composable_node("rtabmap_slam", "rtabmap_slam::CoreWrapper", self.opts_rtabmap)