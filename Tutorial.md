# Overview
This tutorial will walk you through setting up a gazebo simulation of the Dataspeed Inc. Mobility Base and Rethink Robotics Baxter.

# Workspace setup
Note: The following commands must be run from the root of a catkin workspace, or an empty folder you wish to become a cakin workspace.

Initialize workspace
```bash
wstool init src
```

Download required source code
```bash
wget https://bitbucket.org/DataspeedInc/mobility_base_ros/raw/master/mobility_base.rosinstall -O /tmp/mobility_base.rosinstall
wget https://bitbucket.org/DataspeedInc/mobility_base_simulator/raw/master/mobility_base_simulator.rosinstall -O /tmp/mobility_base_simulator.rosinstall
wstool merge -t src /tmp/mobility_base.rosinstall
wstool merge -t src /tmp/mobility_base_simulator.rosinstall
wstool update -t src
```

Install dependencies
```bash
sudo apt-get install ros-$ROS_DISTRO-baxter-simulator ros-$ROS_DISTRO-joint-state-controller
rosdep update && rosdep install --from-paths src --ignore-src -r
```
Build
```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
```

# Run the simulation
If you wish to simulate the Mobility Base without Baxter, use ```baxter:=false```
```bash
source devel/setup.bash
roslaunch mobility_base_gazebo mobility_base_empty_world.launch baxter:=true
```

# Things to try in a new tab
Remember to source the workspace for each new tab ```source devel/setup.bash```

* Enable Baxter
    * ```rosrun baxter_tools enable_robot.py -e```
    * ```rosrun baxter_tools tuck_arms.py -u```
* Visualize sensors in rviz
    * ```roslaunch mobility_base_tools rviz.launch```
* Basic motion commands: [Video](https://www.youtube.com/watch?v=oF-VAFTKkto)
    * ```rosrun mobility_base_examples motion_demo.py```
* Include LIDAR sensors in the simulation
    * Set desired environment variables documented [here](http://mbsdk.dataspeedinc.com/Config).
