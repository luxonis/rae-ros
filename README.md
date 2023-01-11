# rae
(Will be moved to public later)

So far only developed for ROS2 Humble. Will likely work on all recent versions of ROS2. 


### Setting up procedure


1. `mkdir -p rae_ws/src`
2. `cd rae_ws/src`
3. `git clone https://github.com/luxonis/depthai-ros.git`
4. `cd ../..`
5. `rosdep install --from-paths src --ignore-src -r -y`
6. `source /opt/ros/<ros-distro>/setup.bash`
7. `colcon build` 
8. `source install/setup.bash`

State publisher:

```
ros2 launch depthai_bridge rae_desc_launch.py 
```

`rae_desc_launch.py` will use information in `rae.urdf.xacro` to create state publisher for rae.
You can observe the robot with rviz2 using `show_model.rviz` config file that is located in 'depttai_bridge/rviz'. 

```
ros2 launch depthai_bridge rae_gazebo_desc_launch.py 
```
Will do the similar thing, but it will load some gazebo plugins along the way. 

#TODO Add gazebo paths so gazebo knows where to find both map and robot meshses. Right now it assumes it runs inside rae folder

1. `cd src/rae`
2. `ros2 launch depthai_bridge rae_teleopt_wip_launch.py`

That launch file will:
1. Run Ign (gazebo) fortress
2. Run state publisher with gazebo plugins
3. Spawn rae into a world
4. Create ROS2 bridge that is neccesary to send commands via ROS2

You can controll the robot via keyboard teleopt (in a new terminal) with:
1. `sudo apt-get install ros-<ros-distro>-teleop-twist-keyboard
2. ros2 run  teleop_twist_keyboard teleop_twist_keyboard
