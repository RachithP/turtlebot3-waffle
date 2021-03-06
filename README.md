
# TURTLEBOT3 - WAFFLE

**ROS - Kinetic </br>
Ubuntu - 16.04 </br>
Python - 2.7 </br>
Gazebo - 7.0.0** </br>

This package contains instructions to run the Gazebo 7.0.0 simulation of TurtleBot3-waffle on Ubuntu 16.04 and python 2.7
The e-manual for turtlebot3 can be found [here](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/).

Make sure you have installed the following packages:
NOTE: Most of them would be in your system when you installed ros-kinetic (if desktop-full). You can choose not to install any of the below, in that case, no guarantee that the simulation might work :P
```bash
sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-compressed-image-transport ros-kinetic-rqt-image-view ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers ros-kinetic-turtlebot3-gazebo  ros-kinetic-turtlebot3-description
```

After successful installation of the above packages, create a workspace for this simulation.

```bash
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src/
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone https://github.com/RachithP/turtlebot3-waffle.git
cd ~/turtlebot3_ws && catkin_make
source ~/turtlebot3_ws/devel/setup.bash
export TURTLEBOT3_MODEL=waffle
```
Make sure the permissions of executable python files are proper.
```bash
chmod a+x ~/turtlebot3_ws/src/turtlebot3-waffle/scripts/*.py
```

Close the terminal and open a new terminal and run the following:
```bash
roslaunch waffle_rrl_lab waffle_gazebo.launch start_x_pos:=2 start_y_pos:=1 end_x_pos:=7 end_y_pos:=7
```
Enjoy playing with the simulation! </br>
NOTE: The turtlebot may crash into one of the obstacles! Tune parameters in the ```a*_tb3_waffle.py``` and ```turtlebot3_controller.py``` to get it working as you want.

You can also refer to [astar for turtlebot3](https://github.com/RachithP/motion-planning/tree/master/A-star/non-holonomic_robot) for more details on the implementation of astar algorithm.
