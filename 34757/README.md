# MATLAB/ROS2 repository Unmanned Autonomous Systems Course (34757) of Technical University of Denmark

## Pre-requisites
  * Linux OS
  * Matlab 2024b with following toolboxes
    - MATLAB Coder
    - MATLAB Compiler
    - MATLAB Compiler SDK
    - MATLAB Support for MinGW-w64 C/C++ Compiler (example for Windows)
    - Optimization Toolbox
    - ROS Toolbox
    - Robotics System Toolbox
    - Stateflow
    - Simulink
    - Simulink Compiler
    - Simulink Coder
    - Simulink Code Inspector
    - Simulink Check
    - Simulink 3D Animation
    - Simscape
    - Simscape Multibody
    - UAV Toolbox
    - Curve Fitting Toolbox
    - traj_gen-matlab (only through add-on manager)
  * Docker
  
If you have Windows/Mac OS, then use virtual machine to deploy a linux OS. Then follow the rest of instructions as it is.

## Setting-up
```
mkdir uas34757
cd uas34757
docker pull iamkashish/uas34757
git clone https://github.com/IMRCLab/crazyswarm2
```
Then open Matlab inside folder uas34757, and execute command: (Note: `'crazyswarm2'` is a relative path)
```
ros2genmsg('crazyswarm2')
```

After it builds the msg and srv for Matlab based on ROS2 package. You are good to go!

## Usage
Run the docker
```
docker run -it --network host   --env DISPLAY=$DISPLAY   --env QT_X11_NO_MITSHM=1   --env DBUS_SYSTEM_BUS_ADDRESS=unix:path=/run/dbus/system_bus_socket   --volume /tmp/.X11-unix:/tmp/.X11-unix   --volume /dev:/dev   --volume /run/dbus/system_bus_socket:/run/dbus/system_bus_socket   --privileged   iamkashish/uas34757:v1.0
```

Setting up your Crazyflie, edit the `crazyflie.yaml` with your crazyflie name and radio address. Both of the fields should be different for each team/crazyflie.
```
gedit crazyflie_ws/src/crazyswarm2/crazyflie/config/crazyflies.yaml
```
> Note: Crazyflie Name should be `cf<your_team_number>` and radio:`//0/<your_team_number>/2M/E7E7E7E7E7`
> The rigid body name in the OptiTrack of your crazyflie should be same as in crazyflie.yaml file, which is `cf<your_team_number>`.s

Example of lines in <crazyflies.yaml> file:
```
# named list of all robots
robots:
  cf231:                   # Crazyflie Name
    enabled: true
    uri: radio://0/80/2M/E7E7E7E7E7   # Radio address
```

Check connection to the crazyflie:
```
cfclient
```

Run ROS2 and connect the crazyflie:
```
ros2 launch crazyflie launch.py
```

Note: Make sure you are connected to asta-optitrack wifi.

You can use MATLAB or Python to fly your crazyflie. 

For MATLAB:
Make sure that you change topic names in MATLAB simulink file according to your crazyflie name.
Change topics starting from `/cf231/..` to `/cf<your_team_number>/..`