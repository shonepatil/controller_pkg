# ROS2 Controller Package for Self Driving Car

Given coordinates of a path to follow, this ROS2 package allows the car to follow the path using controlers (eg. PID and LQR). We primarily used [f1tenth simulator](https://github.com/f1tenth/f1tenth_gym_ros) to develop our controller package, and the GIF below if from the RViz on the simulator. [Here](https://docs.google.com/document/d/1p193sd0xH9MkXmNO3VeS7I8w-1QvZ5OEhzpqFabwr0k/edit?usp=sharing) is our report, and [here](https://docs.google.com/presentation/d/1H5FHX5CmrK70eJTGlJmzeryEGKBkP8fqUc0uEpG2vxo/edit?usp=sharing) is our presentation.

![PID Controller GIF](./recordings/Robocar_Going_Around_Fast.gif)

# Installation

## System Requirement

We recommend using Linux system to run this ROS2 package.

To run this package on f1tenth simulator, you would need the following:

- [**F1tenth simulator**](https://github.com/f1tenth/f1tenth_gym_ros): You would need to set up a simulation on your own environment by running the docker container. You can find the instruction on their repository.

- [**Path package**](https://github.com/shonepatil/path_pkg): This package is used to provide path node, which the controller package can subscribe to.

## How to run PID Controller

First, git clone this package to appropriate src folder. Then, compile the controller package in ROS2.
```
colcon build --packages-select controller_pkg
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
```
Then, you can launch PID controller by running the following command.
```
ros2 launch controller_pkg pid_launch.launch.py
```

# Overview

## Input (Subscribing)

- **Localization data**

The car takes in the $x$ and $y$ coordinates of the position, which is used to calculate the Cross Track Error.

- **Path data**

The controller subscribes to a path node, and it takes in $x$ and $y$ coordinates of the path the car needs to follow.

## Output (Publishing)

- **Acceleration**

- **Steering**

# PID Controller

A [PID Controller](https://en.wikipedia.org/wiki/PID_controller#:~:text=A%20proportional%E2%80%93integral%E2%80%93derivative%20controller,continuously%20calculates%20an%20error%20value) is a simple controller that makes use of the Cross Track Error, which is the distance between the car and the path the car needs to follow.

```
ros2 launch controller_pkg pid_launch.launch.py
```

![PID Controller Left Turn](./recordings/Robocar_Left_Turn.gif)

You could tune the PID parameters as well as speed in ```pid_config.yaml``` to accomodate for your path.

# LQR Controller

A [LQR Controller](https://en.wikipedia.org/wiki/Linear%E2%80%93quadratic_regulator) is a more advanced controller system with more parameters. While the controller still takes into account of the Cross Track Error, it also considers the orientation of the car.

```
ros2 launch controller_pkg lqr_launch.launch.py
```

![LQR Controller GIF](./recordings/LQRF1TENTH.gif)

# Reference

- [**UCSD Robocar Control Package**](https://gitlab.com/ucsd_robocar2/ucsd_robocar_control2_pkg): We modified this pakcage to meet our own needs.
- [**UCSD Robocar Path Package**](https://gitlab.com/ucsd_robocar2/ucsd_robocar_path2_pkg)