import depthai as dai


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
        self.dai_node = dai.ros.ROSNode("dai", dai.ros.ROSNodeOptions(False,'/dai', '', {}))
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
        self.ros_context_manager.add_node(self.dai_node)
        self.ros_context_manager.spin()
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
        