The project was built on Ubuntu 20.04 and ROS Noetic. The first part comprises of installing all the required dependencies as shown below. 

## Instructions to install dependencies
```bash
$ sudo apt update
$ sudo apt install ros-noetic-joy ros-noetic-octomap-ros ros-noetic-mavlink
$ sudo apt install ros-noetic-octomap-mapping ros-noetic-control-toolbox
$ sudo apt install python3-vcstool python3-catkin-tools protobuf-compiler libgoogle-glog-dev
$ rosdep update
$ sudo apt-get install ros-noetic-ros libgoogle-glog-dev
```

A new workspace is created and all the required ROS packages are downloaded.

## Setup Workspace
```bash 
$ mkdir -p ~/rbe502_project/src
$ cd ~/rbe502_project/src
$ catkin_init_workspace # initialize your catkin workspace
$ cd ~/rbe502_project
$ catkin init
$ cd ~/rbe502_project/src
$ git clone -b dev/ros-noetic https://github.com/gsilano/CrazyS.git
$ git clone -b med18_gazebo9 https://github.com/gsilano/mav_comm.git
```
Note: a new ROS workspace is needed for the project, because the CrazyS Gazebo package is built using the catkin build tool, instead of catkin_make.

python_catkin_tools is used to build the project workspace and it is configured using the following commands:
```bash
$ cd ~/rbe502_project
$ rosdep install --from-paths src -i
$ rosdep update
$ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=False
$ catkin build
```

Do not forget to add sourcing to your .bashrc file:
```bash 
$ echo "source ~/rbe502_project/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

With all dependencies ready, we can build the ROS package by the following commands:
```bash
cd ~/rbe502_project
catkin build
```
To spawn the quadrotor in Gazebo, we can run the following launch file:
```bash 
$ roslaunch rotors_gazebo crazyflie2_without_controller.launch
```

## Problem Statement
Design a sliding mode controller for altitude and attitude control of the Crazyflie 2.0 to enable the quadrotor to track desired 
trajectories and visit a set of desired waypoints. The main components of the project are described below.

**Part 1.** : The quadrotor is supposed to visit the following points one by one   
- p0 = (0, 0, 0) to p1 = (0, 0, 1) in 5 seconds
- p1 = (0, 0, 1) to p2 = (1, 0, 1) in 15 seconds
- p2 = (1, 0, 1) to p3 = (1, 1, 1) in 15 seconds
- p3 = (1, 1, 1) to p4 = (0, 1, 1) in 15 seconds
- p4 = (0, 1, 1) to p5 = (0, 0, 1) in 15 seconds

The sequence of visiting the waypoints does matter. The velocity and acceleration at each waypoint must be equal to zero.

**Part 2.** : A boundary layer-based sliding mode control laws is designed for the z, ϕ, θ, ψ coordinates of the quadrotor to track desired trajectories
zd, ϕd, θd, and ψd.

For this project, the desired yaw angle ψ, and also the desired angular velocities ˙ϕ, ˙ θ, ˙ψ and the desired angular accelerations ¨ϕ, ¨θ, ¨ ψ can be considered zero during the motion, i.e: ψd = 0 and ˙ϕd = ˙ θd = ˙ψd = 0 and ¨ϕd = ¨θd = ¨ ψd = 0
The resulting discrepancy can be considered as an external disturbance that is handled through the robust control design in this project.

**Part 3.** : A ROS node is implemented in Python to evaluate the performance of the
control design on the Crazyflie 2.0 quadrotor in Gazebo. The script must implement the
trajectories generated in Part 1 and the sliding mode control laws formulated in Part 2.

**Part 4.** Once the program is shut down, the actual trajectory is saved into a log.pkl file under
the scripts directory. Run the visualize.py script above to help visualize the trajectory from
the saved log.pkl file.

## Performance Testing in Gazebo
- Open a terminal in Ubuntu, and spawn the Crazyflie 2.0 quadrotor on the Gazebo simulator :
```bash
$ roslaunch rotors_gazebo crazyflie2_without_controller.launch
``` 
Note that the Gazebo environment starts in the paused mode, so make sure that you start the
simulation by clicking on the play button before proceed.
- We can now test the control script developed in Part 3 by running the script in a new terminal.
The quadrotor must be controlled smoothly (no overshoot or oscillations) to track the trajectories
generated in Part 1 and reach the five desired waypoints.
- After the simulation is over, run the visualization script to plot the actual trajectories on top
of the reference trajectories in 3D to ensure a satisfactory control performance.

## Trajectory Image

