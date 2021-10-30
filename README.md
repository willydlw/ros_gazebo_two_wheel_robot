# Differential Drive Robot Simulation Using ROS and Gazebo

This learning unit uses ROS (Robot Operating System) features and tools to study the motion behavior of a two wheeled, differential drive, mobile robot.

Objective: model a simple differential drive robot and configure it to navigate autonomously in a simulation environment.


Thank and acknowledge

The Gazebo, Make a Mobile Robot tutorial, http://gazebosim.org/tutorials/?tut=build_robot, 


The Construct, Exploring ROS using a 2 Wheeled Robot, https://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/
</br></br>


### Test Environment

This project was developed and tested with the following software.

- ROS: Melodic (Desktop Full Version)
- Ubuntu: 18.04
- Gazebo: 9.0.0
- Python: 3.6.9


#### Simulation Workspace Folder
    .
    └── simulation_ws           # Simulation Workspace
        ├── src                 # Source files 
        │   ├── m2wr_description       # Robot definition directory
        |   |   ├── CMakeLists.txt
        |   |   ├── package.xml
        |   |   ├── launch              # launch files directory
        |   |   ├── urdf                # xacro, gazebo files directory

        │   ├── my_worlds               # World definitions
        │   │   └── ...
        │   └── ...
        ├── devel               # Folder created after compilation
        │   ├── setup.bash      # Shell Script to add environment variables to your path
        │   └── ...             # etc.
        └── build               # Compiled files
    

### Terminology

**Robot Operating System (ROS)** "The Robot Operating System (ROS) is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms." [1](https://www.ros.org/about-ros/) It is not truly an operating system. It is an open-source middle-ware robotics framework that provides services such as hardware abstraction, low-level device control, message passing and package management.

**Gazebo** is an open-source 3D, physics-based robot simulator that interfaces with ROS.

**Differential Drive Robot** is typically a two or four wheeled robot that is driven by controlling the wheel velocities.

**URDF (Universal Robot Description Format)** is a format used to describe physical, visual, collison, transmission properties of a robot. Typically uses XML or XACRO notation.

**Plugin** is a piece of software that can be plugged into an existing software framework. Plugins let users integrate new software functionality without the need to re-compile the whole framework.

### Setup

After downloading the files, open a terminal and navigate to the project folder ros_gazebo_two_wheel_robot. For example, if the project folder is stored in your home directory, use the command `cd ~/ros_gazebo_two_wheel_robot`.

Next, we need to build code in the simulation_ws directory with the catkin_make command.

Run the commands below to navigate to the simulation workspace directory, simulation_ws and use the catkin_make tool to build needed project code for the package named m2wr_description.

```bash
cd simulation_ws
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```

After running catkin_make, you should notice two new folders in the root of your simulation workspace: the build and devel folders. The build folder is where cmake and make are invoked, and the devel folder contains any generated files and targets, plus setup.*sh files. It also creates a CMakeLists.txt link in the src folder.

Now we are ready to work through this unit's lessons.</br></br>

## Lessons

[Lesson 0 - Creating a Robot Model](./lesson0/lesson0.md)

[Lesson 1 - Visualizing the robot with rviz](./lesson1/lesson01.md)

[Lesson 2 - Simulating robot movement in an empty world](./lesson2/lesson2.md)

