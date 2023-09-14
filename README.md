# RAE ROS

This repository contains rae [ROS](https://www.ros.org/) integration files.

### Setting up procedure

#### SSH

1. Connect via USB cable or wifi `rae-<ID>`, password `wifiwifi@` (See [rae getting started documentation](https://docs.luxonis.com/projects/hardware/en/latest/pages/rae/#getting-started)).
2. To use SHH without typing password each time - `ssh-copy-id root@192.168.11.1`
3. Currently date resets after each startup to set current - ssh root@192.168.11.1 sudo date -s @`( date -u +"%s" )`

#### Generating docker image
1. Clone repository `git clone git@github.com:luxonis/rae-ros.git`
2. Build docker image `cd rae && docker buildx build --platform arm64 --build-arg USE_RVIZ=0 --build-arg SIM=0 --build-arg ROS_DISTRO=humble --build-arg CORE_NUM=10 -f Dockerfile --squash -t rae_full --load .
`
3. Upload docker image to robot. Note that currently space on the robot is limited, so you need to have 7-8 GB of free space in `/data` directory - `docker save rae_full | ssh -C root@192.168.11.1 docker load`
4. SSH into robot and run docker image - `docker run -it --restart=unless-stopped -v /dev/:/dev/  --privileged  --net=host rae_full`
5. Search for docker container name with `docker ps`
6. Attach to the shell - `docker attach <container_name>`, or if you want to create separate session `docker exec -it <container_name> zsh
7. To launch robot hardware - `ros2 launch rae_bringup robot.launch.py`. This launches:
   - Motor drivers and differential controller
   - Camera driver, currently set up to provide Depth and streams from left & right camera. Note here that you have to calibrate cameras (see steps below). Currently a default calibration file is loaded. It's located in `rae_camera/config/cal.json`. To use one on the device or from other path, change `i_external_calibration_path` parameter in  `rae_camera/config/camera.yaml`
   - Depth image -> LaserScan conversion node used for SLAM
8. Launching whole stack - `ros2 launch rae_bringup bringup.launch.py`. It has following arguments used for enabling parts of the stack:
   - `enable_slam_toolbox` (true)
   - `enable_rosbridge` (false)
   - `enable_rtabmap` (false)
   - `enable_nav` (false)
Example launch with an argument - `ros2 launch rae_bringup bringup.launch.py enable_nav:=false`


#### Calibration

Every shipped rae has already been factory calibrated, so this step is rarely needed. Besides the section below, [Calibration documentation](https://docs.luxonis.com/projects/hardware/en/latest/pages/guides/calibration/) is also a good source of information.

Within the docker image, you can execute:

```bash
apt update
apt install neovim libgl1-mesa-glx python3-pip
git clone --branch rae-calib https://github.com/luxonis/depthai.git
cd depthai/
python3 install_requirements.py
# To calibrate rae's front cameras - for back cameras we would change the board name to "RAE-D-E"
python3 calibrate.py -s <size> -db -nx <squares_X> -ny <squares_Y> -brd RAE-A-B-C -cd 1 -c 3
```

#### Some hardware notes:

##### Peripherals:

- LCD node - accepts BGR8 image (best if already resized to 160x80px) on /lcd Image topic
- LED node - Subscribes to /led topic, message type is LEDControl (refer to rae_msgs/msg/LEDControl)
- Mic node - Publishes audio_msgs/msg/Audio (from gst_bridge package) on /audio_in, configuration is S32_LE, 48kHz, 2 channel interleaved
- Speakers node - Subscibes to audio_out to same type as Mic node, configuration is S16_LE, 41kHz, 2 channel interleaved

##### GST-ROS bridge
You can use gst-bridge for testing, for example to play audio on a ros topic:
- `gst-launch-1.0 --gst-plugin-path=install/gst_bridge/lib/gst_bridge/ filesrc location=sample.mp3 ! decodebin ! audioconvert ! rosaudiosink ros-topic="/audio_out"`

- `gst-launch-1.0 --gst-plugin-path=install/gst_bridge/lib/gst_bridge/ rosaudiosrc ros-topic="audio_out" ! audioconvert ! wavenc ! filesink location=mic1.wav`

- `gst-launch-1.0 --gst-plugin-path=install/gst_bridge/lib/gst_bridge/ rosimagesrc ros-topic="/rae/right_front/image_raw" ! videoconvert ! videoscale ! video/x-raw,width=160,height=80 ! fbdevsink`

- `gst-launch-1.0 alsasrc device="hw:0,1" ! audio/x-raw,rate=48000,format=S32LE ! audioconvert ! spectrascope ! videoconvert ! video/x-raw,width=160,height=80 ! fbdevsink`
##### Sensors and sockets
Socket 1 - OV9782
Socket 0 - IMX214
Socket 2 - OV9782
Socket 3 - OV9782
Socket 4 - OV9782

##### Motors:
Motor configuration parameters are provided in `rae_description/rae_control.xacro` file.
Pin numbers shouldn't change between devices, but if that's the case you can edit that file to set new ones.
- PWM pins (speed control):
<param name="pwmL">19</param>
<param name="pwmR">20</param>
- Phase pins (direction control)
<param name="phL">41</param>
<param name="phR">45</param>
- Encoder pins - each motor has A and B pins for encoders.
<param name="enLA">42</param>
<param name="enLB">43</param>
<param name="enRA">46</param>
<param name="enRB">47</param>
- How many encoder tics are there per revolution - this might vary from setup to setup. To verify that, run the controller and rotate a wheel manually. You can see current positions/velocities by listening on `/joint_states` topic - `ros2 topic echo /joint_states`.
<param name="encTicsPerRevL">187</param>
<param name="encTicsPerRevR">187</param>
- Max motor speed in rads/s
<param name="maxVelL">32</param>
<param name="maxVelR">32</param>

Parameters for differential driver controller are present in `rae_hw/config/controller.yaml`. `wheel_separation` and `wheel_radius` parameters might also need tuning depending on the setup.

#### Testing motors

In `rae_hw/test` you can find three scripts that will help you verify that the motors are running correctly. If you want to change arguments, you need to provide all of them.

1. To find out if encoder is working accurately, execute `ros2 run rae_hw test_encoders` and rotate the wheel by 360 degrees. After rotation, encoder readout should be ~2PI. If not, adjust encoder tick per rev parameter.
Scipt arguments - `[encRatioL encRatioR]`. Full arg version `ros2 run rae_hw test_encoders 187 187`
2. Finding out max speed - `ros2 run rae_hw test_max_speed`. Script arguments `[duration encRatioL encRatioR]`. Full arg version `ros2 run rae_hw test_max_speed 1.0 187 187`
3. Motor verification - `ros2 run rae_hw test_motors`. Script arguments `[duration speedL speedR encRatioL encRatioR maxVelL maxVelR]`. Full arg version `ros2 run rae_hw test_motors 5.0 16.0 16.0 187 187 32 32`


You can controll the robot via keyboard teleopt (in a new terminal) with:
1. sudo apt-get install ros-<ros-distro>-teleop-twist-keyboard
2. ros2 run  teleop_twist_keyboard teleop_twist_keyboard
