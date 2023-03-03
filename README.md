# HKCLR-Depalletizing-Demo
Control algorithm of a self-developed depalletizing robot
## Note 
If you have Docker engine, you can skip 'Pre-requisites' and 'Install'. you can start from 'Docker Image' to build the [Dockerfile](Dockerfile) as a Docker Image and run this repo in a Docker container. If you prefer to use this repo without Docker engine, you can ignore Dockerfile and you have to meet the Pre-requests as below: 
## Pre-requests
### For using robot "Without" vision algorithm
 - Ubuntu 18.04
 - ROS Melodic
 - Webots 2020b-rev1 or newer version:  
   If you are new to Webots, copy the link below in a web browser such as Google Chrome, and download the Webots debian package, open it and click 'install'.
   `````
   https://github.com/cyberbotics/webots/releases/download/R2020b-rev1/webots_2020b-rev1_amd64.deb
   `````
- Dependence:
````
  sudo apt-get update && apt-get install -y \
    ros-melodic-moveit \
    ros-melodic-actionlib \
    ros-melodic-actionlib-tutorials \
    ros-melodic-control-msgs \
    ros-melodic-roscpp \
    ros-melodic-behaviortree-cpp-v3 \
    ros-melodic-ros-control \
    ros-melodic-ros-controllers \  
    ros-melodic-industrial* \
    libpcl-dev pcl-tools \
    ros-melodic-pcl* \
````
### For using robot "With" vision algorithm
 - All dependence mentioned above
 - Python3 virtual environment and all python3 dependence for vision algorithm
   Follow steps below:
   1. install virtualenv
   ````
   pip install virtualenv
   ````
   2. setup python3.6 environment
   ````
   mkdir myvenv
   cd myvenv
   virtualenv -p /usr/bin/python3.6 env3.6
   ````
   3. active the python3.6 environment and install ros python3 dependence in this environment
    ````
   source ~/myvenv/env3.6/bin/activate
   ````  
   ````
   pip3 install pyyaml rospkg
   ````
   4. keep installing other dependence of vision algorithm in this viltual python3 env by following the [README.md](depalletizing_ws/src/box_node/boxdemo/README.md)
   
## Install
1. Compile this repo under workspace `~/HKCLR-Depalletizing-Demo/depalletizing_ws`
````
catkin_make
````
2. Source ros, LD_Lib and python path under workspace `~/HKCLR-Depalletizing-Demo/depalletizing_ws` when every new terminal is launched
````
source setup.bash
````
## Docker Image
The [Dockerfile](depalletizing_ws/Dockerfile), which can be used to build a docker image consists of ubuntu18.04, nvidia/cudagl, webots, ros-melodic, moveit, and self-defined ROS packages. 
This is super useful and convenient. Any computer with recent Docker Engine and Nvidia GPU can easily use this repo without installing a ton of dependence. 
Also, it can beyond the limitation of operating system, eg. you can even use this repo on Windows. 
### Creat Image
Using docker without 'sudo' the cmd. If you never done this before, it is suggested that typing in following cmd:
````
sudo groupadd docker
sudo gpasswd -a ${USER} docker
sudo systemctl restart docker
sudo chmod a+rw /var/run/docker.sock
````
Create the image using Dockerfile， Run the cmd under `~/HKCLR-Depalletizing-Demo/depalletizing_ws`  
````
docker build -t webots_moveit_robot .
````
This will create the deployment image named `webots_moveit_robot`

### NVIDIA Container Toolkit
You will need the NVIDIA GPU support for the docker container. To build the bridge, run following cmd:
```
curl https://get.docker.com | sh \
  && sudo systemctl start docker \
  && sudo systemctl enable docker
```
````
distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
   && curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add - \
   && curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
````
````
sudo apt-get update
sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker
````

### Run Image
To run Webots and Rviz with a graphical user interface in a docker container, you need to enable connections to the X server before starting the docker container:
````
xhost +local:root > /dev/null 2>&1
````
Run docker image in container called `hkclr_robot_interface`
````
docker run --gpus=all --privileged --net=host -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw -it --rm --name hkclr_robot_interface webots_moveit_robot bash
````
Tips: You can create `.bash_aliases` file in `~` and add the following to it(optional):
````
alias robot_run="docker run --gpus=all --privileged --net=host -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw -it --rm --name hkclr_robot_interface webots_moveit_robot bash"
````
Such that you only need to issue `robot_run` to launch the program.  

This repo needs many terminals in total, so after you do docker run in one terminal, you have to open other new terminals, and type in following cmd to connect them with the first one you opened by the same container `hkclr_robot_interface` 
````
docker exec -it hkclr_robot_interface bash
````

## Usage
No matter whether you use or not use docker, following part are the same.
## Simulation
````
source setup.bash
````
### Launch webots world with SelfRobot and its drivers
````
roslaunch robot_webots robot_webots_simulation.launch
````
### For Aubo robot, launch moveit move_group with kdl kinematics plugin and move_group_interface server
````
roslaunch robot_controller moveit_group_aubo.launch use_sim_time:=true
````
### For SelfDeveloped robot, launch moveit move_group with ikfast kinematics plugin and move_group_interface server
````
roslaunch robot_controller moveit_group_robot.launch use_sim_time:=true
````
### Launch the depalletizing demo
````
roslaunch roport hkcenter_depalletizing_simulation_aubo.launch use_sim_time:=true
````

## Real Robot
#### (Optional) Launch ros controller of SelfRobot and its hardware interface
Note: If you have self-defined action server and joint states pulisher, you can not launch this!!!    
This ia a fake controller to visualize the movement of robot under motion planning algorithm so we can debug easily.
````
roslaunch robot_controller ArmController.launch
````
### If without vision
#### 1. Launch real driver of SelfRobot
````
rosrun robot_controller curi_ros_driver.py /home/mo/Desktop/cr2_udp_config.json
````
#### 2. For SelfDeveloped robot, launch moveit move_group with ikfast kinematics plugin and move_group_interface server
````
roslaunch robot_controller moveit_group_robot.launch use_sim_time:=false
````
Now you can control real SelfRobot by MontionPlanning Plugin in Rviz
#### 3. Defined the goal points in [depalletizing_helper_for_real.py](depalletizing_ws/src/roport/scripts/depalletizing_helper_for_real.py)
The robot has its workspace limitation, so make sure your self defined goal points are within conditions below:
-  x has to be in 0.83 to 0.91m
-  |y| has to be smaller than 0.95m, if y >= 0, is on left; if y<0, is on right.
-  z has to be within 0.49m to 2.424m
#### 4. running motion planning pipeline
Before running following cmd, make sure that the x coordinate (wrt. base) of robot's gripper is less than 0.82, so that it can initialize successfully and start working. The reason of this is to prevent gripper from collision with boxes in front when initialize.
````
roslaunch roport hkcenter_depalletizing_real.launch use_sim_time:=false
````
### If with vision
#### 1. Launch real driver of SelfRobot and Aubo
````
rosrun robot_controller curi_ros_driver.py /home/mo/Desktop/cr2_udp_config.json
````
````
roslaunch aubo_i10_moveit_config moveit_planning_execution.launch sim:=false robot_ip:=192.168.0.7
````
#### 2. For SelfDeveloped robot, launch moveit move_group with ikfast kinematics plugin and move_group_interface server
````
roslaunch robot_controller moveit_group_robot.launch use_sim_time:=false
````
````
roslaunch robot_controller moveit_group_aubo.launch use_sim_time:=false
````
#### 3. For smarteye camera, launch smarteye node to be used for geting pointcloud and 2D image
````
roslaunch smarteye smarteye.launch
````
#### 4. For BoxDemo Detection Algorithm (Must under Python3 virtual environment), launch box node to process the incoming pointcloud and 2D image
the BoxDemo Detection Algorithm currently has 2 modes, you can switch the mode in [run_box_node.py](depalletizing_ws/src/box_node/script/run_box_node.py):
-  if self.is_pose_single_ = True, it will detect only one pose which is the closest one from camera
-  if self.is_pose_single_ = False, it will detect poses as much as possible. Then the motion planning part will do filtering and choose to pick the optimal pose.  

you can choose whether to display the detection result in [run_box_node.py](depalletizing_ws/src/box_node/script/run_box_node.py):
-  if self.display_result_ = True, it will prompt the visualization window showing coordinates on point cloud
-  if self.display_result_ = False, it will not prompt the visualization window 
````
rosrun box_node run_box_node.py
````
(Optional) You can use simulated data sender to validate the perception pipeline by following under python2 environment:
````
rosrun box_node send_data_to_box_node.py
````
#### 5. For hand eye calibration, make sure you have figured out the self.hand_eye_relationship_ in [depalletizing_helper_for_real.py](depalletizing_ws/src/roport/scripts/depalletizing_helper_for_real.py)
hand_eye_relationship x, y, z, ox, oy, oz, ow, need to be defined or get when run time
#### 6. running motion planning pipeline
Before running following cmd, make sure that the x coordinate (wrt. base) of robot's gripper is less than 0.82, so that it can initialize successfully and start working. The reason of this is to prevent gripper from collision with boxes in front when initialize.
````
roslaunch roport hkcenter_depalletizing_real.launch use_sim_time:=false
````