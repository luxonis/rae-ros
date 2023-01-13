# rae
(Will be moved to public later)

So far only developed for ROS2 Humble. Will likely work on all recent versions of ROS2. 

Make sure you have [IGN (Gazebo) Fortress](https://gazebosim.org/docs/fortress/install) installed, along with [ros_ign_bridge](https://github.com/gazebosim/ros_gz/tree/ros2#from-source) for Fortress version of IGN Gazebo (for ROS2 Humble distro). 

### Setting up procedure


1. `mkdir -p rae_ws/src`
2. `cd rae_ws/src`
3. `git clone git@github.com:luxonis/rae.git` 
4. `cd ..`
5. `rosdep install --from-paths src --ignore-src -r -y`
6. `source /opt/ros/<ros-distro>/setup.bash`
7. `colcon build` 
8. `source install/setup.bash`


```
ros2 launch rae_gazebo rae_simulation.launch.py 
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
