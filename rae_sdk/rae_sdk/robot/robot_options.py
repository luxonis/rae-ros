class RobotOptions:
    """
    A class for storing the robot's options.

    Attributes
    ----------
        name (str): The robot's name.
        namespace (str): The robot's namespace.
        launch_controllers (bool): Whether to launch the robot's controllers.
        start_hardware (bool): Whether to start the robot's hardware.
        launch_mock (bool): Whether to launch the robot's mock interfaces if start_hardware=True.
        publish_state_info (bool): Whether to publish state information.

    """

    def __init__(self, name='rae_api', namespace='', launch_controllers=True, start_hardware=True, launch_mock=False, publish_state_info=True):
        self._start_hardware = start_hardware
        self._launch_mock = launch_mock
        self._name = name
        self._namespace = namespace
        self._launch_controllers = launch_controllers
        self._publish_state_info = publish_state_info

    @property
    def start_hardware(self):
        return self._start_hardware

    @property
    def launch_mock(self):
        return self._launch_mock

    @property
    def name(self):
        return self._name

    @property
    def namespace(self):
        return self._namespace

    @property
    def launch_controllers(self):
        return self._launch_controllers
    @property
    def publish_state_info(self):
        return self._publish_state_info
