# RAE ROS

This repository contains rae [ROS](https://www.ros.org/) integration files. 
**Note** The RAE project is under active development so you can expect changes in the API and improvements in the performance in near future. Please report problems on Github issues as it's important for the development efforts.

### Setting up procedure

#### SSH

1. Connect via USB cable or wifi `rae-<ID>`, password `wifiwifi@` (See [rae getting started documentation](https://docs.luxonis.com/projects/hardware/en/latest/pages/rae/#getting-started)).
2. To use SHH without typing password each time - `ssh-copy-id root@192.168.11.1`
3. Currently date resets after each startup to set current - ssh root@192.168.11.1 sudo date -s @`( date -u +"%s" )`
4. If you want to run ROS packages while bypassing RobotHub it would be advised to stop RH agent before starting docker containers, otherwise you can easily run into conflicts as they would be competing for same hardware resources - `robothub-ctl stop`. **Keep in mind** that since `wpa_supplicant` is a subproccess of the RH agent, the WiFi connection will get killed along with the agent. To resolve this we recommend you manually setup the WiFi connection as done in [this guide](https://docs-beta.luxonis.com/deploy/connect-device/rae/?v=Advanced+%28manual%29).


#### Generating docker image

You can download prebuilt images form [dockerhub](https://hub.docker.com/r/luxonis/rae-ros-robot/tags), in which case you can skip first 2 steps in guide below. We reccomend using image tagged as humble as all other images are generally experimental images. You can download docker image with: 

`docker pull luxonis/rae-ros-robot:humble`

Downloading prebuilt images is reccomended if you are not planning to considerably change source code. 

1. Clone repository `git clone git@github.com:luxonis/rae-ros.git`
2. Build docker image `cd rae && docker buildx build --platform arm64 --build-arg USE_RVIZ=0 --build-arg SIM=0 --build-arg ROS_DISTRO=humble --build-arg CORE_NUM=10 -f Dockerfile --squash -t <docker-image-name>:<tag> --load .
`
3. Upload docker image to robot. Connect robot to your PC via USB so you can transfer image quicker. Note that currently space on the robot is limited, so you need to have 7-8 GB of free space in `/data` directory - `docker save <docker-image-name>:<tag> | ssh -C root@192.168.197.55 docker load`
4. SSH into robot and run docker image - `docker run -it --restart=unless-stopped -v /dev/:/dev/ -v /sys/:/sys/ --privileged  --net=host <docker-image-name>:<tag>`
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

#### Testing motors

In `rae_hw/test` you can find three scripts that will help you verify that the motors are running correctly. If you want to change arguments, you need to provide all of them.

1. To find out if encoder is working accurately, execute `ros2 run rae_hw test_encoders` and rotate the wheel by 360 degrees. After rotation, encoder readout should be ~2PI. If not, adjust encoder tick per rev parameter.
Scipt arguments - `[encRatioL encRatioR]`. Full arg version `ros2 run rae_hw test_encoders 756 756`
2. Finding out max speed - `ros2 run rae_hw test_max_speed`. Script arguments `[duration encRatioL encRatioR]`. Full arg version `ros2 run rae_hw test_max_speed 1.0 756 756`
3. Motor verification - `ros2 run rae_hw test_motors`. Script arguments `[duration speedL speedR encRatioL encRatioR maxVelL maxVelR]`. Full arg version `ros2 run rae_hw test_motors 5.0 16.0 16.0 756 756 32 32`

##### Motors:
Motor configuration parameters are provided in `rae_description/urdf/rae_ros2_control.urdf.xacro` file.
Pin numbers shouldn't change between devices, but if that's the case you can edit that file to set new ones.
- PWM pins (speed control):
```
<param name="pwmL">2</param>
<param name="pwmR">1</param>
```
- Phase pins (direction control)
```
<param name="phL">41</param>
<param name="phR">45</param>
```
- Encoder pins - each motor has A and B pins for encoders.
```
<param name="enLA">42</param>
<param name="enLB">43</param>
<param name="enRA">46</param>
<param name="enRB">47</param>
```
- How many encoder tics are there per revolution - this might vary from setup to setup. To verify that, run the controller and rotate a wheel manually. You can see current positions/velocities by listening on `/joint_states` topic - `ros2 topic echo /joint_states`.
```
<param name="encTicsPerRevL">756</param>
<param name="encTicsPerRevR">756</param>
```
- Max motor speed in rads/s
```
<param name="maxVelL">32</param>
<param name="maxVelR">32</param>
```

- Both wheels have parameters for PID control set in that file, those values could need some tuning:
  ```
      <param name="closed_loopR">1</param>
      <param name="PID_P_R">0.2</param>
      <param name="PID_I_R">0.1</param>
      <param name="PID_D_R">0.0005</param>
   ```

Parameters for differential driver controller are present in `rae_hw/config/controller.yaml`. `wheel_separation` and `wheel_radius` parameters might also need tuning depending on the setup.

Implementation of motor control is found in rae_hw package. You can set the motor to be ready to recieve twist commands on /cmd_vel topic by running:
 
 `ros2 launch rae_hw control.launch.py`

You can then control the robot via keyboard teleopt from your pc via (assuming you are connected to same network robot is in):
1. sudo apt-get install ros-humble-teleop-twist-keyboard
2. ros2 run  teleop_twist_keyboard teleop_twist_keyboard

If keyboard is too limitng for your tests you could also use ros-humble-teleop-twist-joy and connect a joystick. 

#### LED node

Under rae_msgs package you can find custom messages that let you control LED lights around the robot. Under those messages there are 3 control types: 

1. Control all (set control_type to 0) gives all LEDs the same color
2. Control single (control_type to 1) lets you control just a single LED light by setting single_led_n variable
3. Custom control (control_type to 2) where you send a list that defines every value of LED lights at once.

Useful example of how to work with LEDs can be found in rae_bringup/scripts/led_test.py where this for loop is populating LED values (with custom control) for each individual LED: 

```
 for i in range(40):
            led_msg.single_led_n = 0
            led_msg.control_type = 2 
            if i < 8:
                color = "white"
                led_msg.data[i]=(colors[color])
            if i >9 and i < 14 and angular_speed > 0.0 and blinking==True:
                color = "yellow"
                led_msg.data[i]=(colors[color])
            if i > 20 and i < 29 and linear_speed < 0.0:
                color = "red"
                led_msg.data[i]=(colors[color])
            if i> 34 and i < 39 and angular_speed < 0.0 and blinking==True:
                color = "yellow"
                led_msg.data[i]=(colors[color]) 
```
We can then send that message to a topic that LED node listens to - by default that is /leds . Easiest way to run LED node (and all other peripheral node) is to run one of the  following launch files: 

``` 
ros2 launch rae_hw peripherals.launch.py
ros2 launch rae_hw control.launch.py
ros2 launch rae_bringup robot.launch.py
```

Where first file is running only peripherals, 2nd is running peripherals and motors, while 3rd is running both of those along with cameras. It could be useful to create your own launch file that mimics peripherals launch file, so you can run your own examples along with nodes. 

### LCD node

LCD node is listening to messages on /lcd topic and showing it on the screen in the. Useful demo for this node is in rae_bringup/scripts/battery_status.py where we subscribe to battery status topic and based on that create image of battery status on LCD screen. This node expects ROS (sensor_msg/Image) image, so you will generally have to transform opencv imaget to ROS image: 

`img_msg = self.bridge.cv2_to_imgmsg(img_cv, encoding="bgr8")`

### Microcphone and speakers

Microphone node expects audio messages and example of how to use that data (along with some other peripherals) can be found in rae_bringup/scripts/audio_spectrum.py . For fair amount of use caes, you will need to decode incoming data as shown in example below.

```
 if msg.encoding == "S32LE":
            audio_data = np.frombuffer(msg.data, dtype=np.int32)
        elif msg.encoding == "S16LE":
            audio_data = np.frombuffer(msg.data, dtype=np.int16)
        if msg.layout == Audio.LAYOUT_INTERLEAVED:
            # Deinterleave channels
            audio_data = audio_data.reshape((msg.frames, msg.channels))
```


Speakers operate similarly, in that they output audio messages. In bringup package in scripts folder sound_test.py offers a decent example of how you can create audio messages. We will shortly create more demos for speakers and microphone. 
