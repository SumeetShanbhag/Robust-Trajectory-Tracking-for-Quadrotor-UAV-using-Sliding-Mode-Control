The project was built on Ubuntu 20.04 and ROS Noetic. The first part comprises of installing all the required dependencies as shown below. 
```bash
$ sudo apt update
$ sudo apt install ros-noetic-joy ros-noetic-octomap-ros ros-noetic-mavlink
$ sudo apt install ros-noetic-octomap-mapping ros-noetic-control-toolbox
$ sudo apt install python3-vcstool python3-catkin-tools protobuf-compiler libgoogle-glog-dev
$ rosdep update
$ sudo apt-get install ros-noetic-ros libgoogle-glog-dev
```

A new workspace is created and all the required ROS packages are downloaded.
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
