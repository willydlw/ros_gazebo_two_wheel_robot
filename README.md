# Create Two-Wheeled ROS robot in simulation

Objectives

- Build a URDF model of asimple two-wheeled differential drive robot
- Specify visualization components so that the model may be viewed in rviz
- Specify Gazebo simulation components to enable control in a 3D simualtion.
</br></br>

## Robot Model

The robot model consists of link elements, joint elements, and gazebo plugins. A link element is a rigid component. Links are attached to other links via joints. The joint elements specify the relative motion between links. Typically, joints allow for rotation or translation.

The common properties specified for links and joints are inertial, collision, and visual. Inertial and collision properties enable physics simulation. The visual properties control the robot's appearance.

ROS uses Unified Robot Description Format (URDF) files to specify a robot's properties. URDF supports XML and xacro (XML macro) languages.

We will build our robot model, step by step, first specifying its visual properties in an XML file. </br></br>

## Create ddbot package

Start by creating a package named ddbot in a catkin workspace with no dependencies. Dependencies will be added later, as needed. Open a terminal and type the following commands.

```bash
cd ~catkin_ws/src
catkin_create_pkg ddbot
```

This will create a /dbot directory with a package.xml file and a CMakeLists.txt.

Next, build the package in the catkin workspace.

```bash
cd ~/catkin_ws
catkin_make
```

</br></br>

Create three new directories in the ddbot package and then navigate to the urdf directory.

```bash
cd ~/catkin_ws/src/ddbot
mkdir launch urdf rviz
cd urdf
```

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



## Lessons

[Lesson 1 - Creating a Robot Model](./lesson1/lesson1.md)

[Lesson 2 - Visualizing the robot with rviz](./lesson2/lesson2.md)

[Lesson 3 - Simulating robot movement in an empty world](./lesson3/lesson3.md)


## Acknowledgements

This information was obtained from the following resources.

ROS Robotics By Example - Second Edition

The Gazebo, Make a Mobile Robot tutorial, http://gazebosim.org/tutorials/?tut=build_robot


The Construct, Exploring ROS using a 2 Wheeled Robot, https://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/