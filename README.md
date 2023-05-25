# RAE-ROS
(Will be moved to public later)

Welcome to official RAE ROS repository.

Make sure you have [IGN (Gazebo) Fortress](https://gazebosim.org/docs/fortress/install) installed, along with [ros_ign_bridge](https://github.com/gazebosim/ros_gz/tree/ros2#from-source) for Fortress version of IGN Gazebo (for ROS2 Humble distro). 


### Setting up procedure - Real Robot
#### SSH

1. Connect via USB cable or wifi `keembay`, password `wifiwifi@`
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
6. To launch robot hardware - `ros2 launch rae_bringup robot.launch.py`. This launches:
   - Motor drivers and differential controller
   - Camera driver, currently set up to provide Depth and streams from left & right camera. Note here that you have to calibrate cameras (see steps below). Currently a default calibration file is loaded. It's located in `rae_bringup/config/cal.json`. To use one on the device or from other path, change `i_external_calibration_path` parameter in  `rae_bringup/config/camera.yaml`
   - Depth image -> LaserScan conversion node used for SLAM
7. Launching navigation stack - on host run `ros2 launch rae_bringup bringup.launch.py sim:=false use_rviz:=true`

#### Calibration
Steps to use it in basic docker image:

- apt update
- apt install neovim libgl1-mesa-glx python3-pip
- git clone --branch rae-calib https://github.com/luxonis/depthai.git
- cd depthai/
- python3 install_requirements.py 
-  python3 calibrate.py -s 2.5 -db -nx 11 -ny 8 -brd /depthai/cal.json -cd 1 -c 3

You might need to create a separate base calibration file for that (cal.json), maybe using rae board will work in your case.
Contents of cal.json
``` json
{
    "board_config":
    {
        "name": "RAE",
        "revision": "R1M0E1",
        "cameras":{
            "CAM_C": {
                "name": "right",
                "hfov": 110,
                "type": "color"
            },
            "CAM_B": {
                "name": "left",
                "hfov": 110,
                "type": "color",
                "extrinsics": {
                    "to_cam": "CAM_C",
                    "specTranslation": {
                        "x": -7.5,
                        "y": 0,
                        "z": 0
                    },
                    "rotation":{
                        "r": 0,
                        "p": 0,
                        "y": 0
                    }
                }
            }
        },
        "stereo_config":{
            "left_cam": "CAM_C",
            "right_cam": "CAM_B"
        }
    }
}
```
Changes to calibrate.py:
add
`imx412' : dai.ColorCameraProperties.SensorResolution.THE_800_P,`
in sensors list (again this might be my setup only)
and raise epipolar error threshold, current max value is 0.6 and if you get more calibration won't save.


#### Some hardware notes:

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

### Setting up procedure - Simulation


1. `mkdir -p rae_ws/src`
2. `cd rae_ws/src`
3. `git clone git@github.com:luxonis/rae.git` 
4. `cd ..`
5. `rosdep install --from-paths src --ignore-src -r -y`
6. `source /opt/ros/<ros-distro>/setup.bash`
7. `colcon build` 
8. `source install/setup.bash`


```
ros2 launch rae_bringup bringup.launch.py use_rviz:=true
```
Launch file will spawn a basic world (with sun and ground plane) with RAE in the middle, along with ROS2 bridge which will send over Twist commands to the simulation. It may take a while for program to start for the first time since it will be downloading RAE model from Fuel. If you want to use local model you can point IGN_GAZEBO_RESOURCE_PATH enivroment variable towards models folder in this package. You can also add this model to any world (defined by a sdf file) with this code snippet, where pose defines a starting position:

```
        <include>
            <pose> 0 0 0.05 0 0 0 </pose>
            <uri>
                https://fuel.gazebosim.org/1.0/danilopejovic/models/WIP-robotmodel1danilo
            </uri>
        </include>
```


You can controll the robot via keyboard teleopt (in a new terminal) with:
1. sudo apt-get install ros-<ros-distro>-teleop-twist-keyboard
2. ros2 run  teleop_twist_keyboard teleop_twist_keyboard
