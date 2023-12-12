import depthai as dai

from .pipeline import rtabmap_pipeline
class PerceptionSystem:
    """
    A class for managing camera functionalities in a robot, interfacing with both depthai and robothub libraries.
    It includes initialization and management of camera streams, publishing capabilities, and camera device control.

    Attributes:
        ros_context_manager (dai.ros.ROSContextManager): Manager for ROS context.
        dai_node (dai.ros.ROSNode): ROS node for depthai operations.
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
        add_ros_stream(stream_name): Adds a ROS stream with the given name.
        add_queue(name, callback): Adds a queue to the device for handling callbacks.
        start_pipeline(pipeline): Starts the camera pipeline and initializes ROS node and context.
        publish_rh(name, color_frame, timestamp, metadata): Publishes video data to RobotHub.
        publish_ros(name, msg): Publishes a message to a ROS topic.
    """
    def __init__(self):
        """
        Initializes the Camera instance.
        """
        self.robothub_available = False
        try:
            import robothub
            self.robothub_available = True
            self.device_mxid = robothub.DEVICES[0].oak["serialNumber"]
            self.device_info = dai.DeviceInfo(self.device_mxid)
            self.rh_stream_handles = {}
        except ImportError:
                print("RobotHub module is not available.")
        self.ros_context_manager = dai.ros.ROSContextManager()
        self.ros_context_manager.init([""], 'single_threaded')
        if not self.robothub_available:
            self.device = dai.Device()
            self.cal_handler = self.device.readCalibration()

        self.pipeline = None
        self.ros_stream_handles = {}
        print("Perception ready")

    def start_ros(self):
        """
        Starts the ROS context manager and spins the ROS node.
        """
        # self.ros_context_manager.add_node(self.dai_node)
        # self.ros_context_manager.spin()
    def stop(self):
        """
        Closes the connection to the depthai device, ensuring a clean shutdown.
        """
        if self.device:
            self.device.close()
        
    def add_rh_stream(self, stream_name):
        """
        Adds a video stream to RobotHub with the specified name.

        Args:
            stream_name (str): The name of the stream to be added.
        """

        if self.robothub_available:
            self.rh_stream_handles[stream_name] = robothub.STREAMS.create_video(
                self.device_mxid, stream_name, stream_name
            )
        else:
            print("RobotHub is not available. Cannot add RobotHub stream.")

    def add_ros_stream(self, stream_name):
        """
        Adds a ROS video stream with the specified name and sets up the necessary configurations.

        Args:
            stream_name (str): The name of the ROS stream to be added.
        """

        self.ros_stream_handles[stream_name] = dai.ros.ImgStreamer(self.dai_node, self.cal_handler, self.stream_name_to_socket(stream_name), stream_name, f"{stream_name}_frame", False, False)
        self.ros_stream_handles[stream_name].convertFromBitStream(dai.RawImgFrame.Type.BGR888i)
    def add_imu_ros_stream(self, stream_name):
        """
        Adds a ROS IMU stream with the specified name and sets up the necessary configurations.

        Args:
            stream_name (str): The name of the ROS stream to be added.
        """
        self.ros_stream_handles[stream_name] = dai.ros.ImuStreamer(self.dai_node, stream_name, "imu_frame", dai.ros.ImuSyncMethod.COPY, 0.0, 0.0, 0.0, 0.0, True, False, False)


    def add_queue(self, name, callback):
        """
        Adds a queue to the depthai device for processing callbacks.

        Args:
            name (str): The name of the queue.
            callback (callable): The callback function to be added to the queue.
        """

        self.device.getOutputQueue(name, 1, False).addCallback(callback)
        
    def start_pipeline(self, pipeline):
        """
        Starts the camera pipeline, initializes the ROS node, and spins the ROS context manager.

        Args:
            pipeline: The pipeline configuration for the camera.
        """

        self.device.startPipeline(pipeline)
        
    def publish_rh(self, name, color_frame, timestamp, metadata):
        """
        Publishes video data to a RobotHub stream.

        Args:
            name (str): The name of the RobotHub stream.
            color_frame: The color frame data to be published.
            timestamp: The timestamp associated with the frame.
            metadata: Additional metadata for the frame.
        """
        if self.robothub_available:
            self.rh_stream_handles[name].publish_video_data(bytes(color_frame), timestamp, metadata)
        else:
            print("RobotHub is not available. Cannot publish to RobotHub.")

        
    def publish_ros(self, name, msg):
        """
        Publishes a message to a ROS topic.

        Args:
            name (str): The name of the ROS topic.
            msg: The message to be published.
        """
        self.ros_stream_handles[name].publish(name, msg)
        
    def stream_name_to_socket(self, stream_name):
        if stream_name == 'stream_front':
            return dai.CameraBoardSocket.CAM_C
        elif stream_name == 'stream_back':
            return dai.CameraBoardSocket.CAM_D
    def setup_perception_rtabmap(self):
        name='/rae'
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
        calHandler = self.device.readCalibration()
        self.opts_rtabmap = dai.ros.ROSNodeOptions(False, "/rtabmap", "/workspaces/rae_ws/test_params.yaml", 
                                              {"odom": "/diff_controller/odom",
                                                "rgb/image": name+"/right/image_rect",
                                                "rgb/camera_info": name+"/right/camera_info",
                                                "depth/image": name+"/stereo/image_raw"})
        # self.rtabmap = dai.ros.RTABMapCoreWrapper(self.opts_rtabmap)
        self.opts = dai.ros.ROSNodeOptions(False, "/dai", "/workspaces/rae_ws/test_params.yaml", {"t":"t"})
        self.dai_node = dai.ros.ROSNode("dai", self.opts)

        self.ros_stream_handles['imu'] = dai.ros.ImuStreamer(self.dai_node, "/rae/imu/data", "rae_imu_frame", dai.ros.ImuSyncMethod.COPY, 0.0, 0.0, 0.0, 0.0, True, False, False)
        self.ros_stream_handles['left'] = dai.ros.ImgStreamer(self.dai_node, calHandler, dai.CameraBoardSocket.CAM_B, "/rae/left/image_rect", "rae_left_camera_optical_frame", 640, 400)
        self.ros_stream_handles['right'] = dai.ros.ImgStreamer(self.dai_node, calHandler, dai.CameraBoardSocket.CAM_C, "/rae/right/image_raw", "rae_right_camera_optical_frame", 640, 400)
        self.ros_stream_handles['stereo'] = dai.ros.ImgStreamer(self.dai_node, calHandler, dai.CameraBoardSocket.CAM_C, "/rae/stereo/image_raw", "rae_right_camera_optical_frame", 640, 400)
        self.device.getOutputQueue("imu", 8, False).addCallback(self.stream_callback_ros)
        self.device.getOutputQueue("stereo", 8, False).addCallback(self.stream_callback_ros)
        self.device.getOutputQueue("left", 8, False).addCallback(self.stream_callback_ros)
        self.device.getOutputQueue("right", 8, False).addCallback(self.stream_callback_ros)
        self.scan_front_opts = dai.ros.ROSNodeOptions(False, "/laserscan_kinect_front", "/workspaces/rae_ws/test_params.yaml", 
                                                 {"laserscan_kinect:__node":"laserscan_kinect_front",
                                                  "/image": name+"/stereo/image_raw",
                                                  "/camera_info": name+"/stereo/camera_info",
                                                  "/scan": name+"/scan_front",
                                                  "/debug_image": name+"/debug_image_front",
                                                  "/debug_image/compressed": name+"/debug_image_front/compressed"})
        self.laserscan_front = dai.ros.LaserScanKinectNode(self.scan_front_opts)

        self.opts_rectify = dai.ros.ROSNodeOptions(False, "/rectify", "/workspaces/rae_ws/test_params.yaml", {"image":"/rae/right/image_raw", "camera_info": "/rae/right/camera_info", "image_rect":"/rae/right/image_rect"})

        self.rectify = dai.ros.ImageProcRectifyNode(self.opts_rectify)
        self.ros_context_manager.add_node(self.dai_node)
        self.ros_context_manager.add_node(self.laserscan_front)
        self.ros_context_manager.add_node(self.rectify)
        # self.ros_context_manager.add_node(self.rtabmap)
        self.ros_context_manager.spin()