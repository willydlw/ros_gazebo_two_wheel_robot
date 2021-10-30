# Lesson 0 - Build Robot Model with URDF

Objectives
- Build a two-wheeled, differential drive robot model
- Visualize model with rviz

This lesson illustrates how to visualize the robot model with rviz.</br></br>

## Create Directory Structure

This example starts by creating a catkin workspace named ddbot_ws. If you already have a catkin workspace, you may prefer using it to create the new package.

Start by creating a directory structure for our workspace and a ROS package named ddbot_description. Open a terminal and type the following commands. 

```bash
mkdir -p ddbot_ws/src
cd ddbot_ws/src/
catkin_create_pkg ddbot_description
```

This will set up the catkin workspace and create a package.xml and CMakeLists.txt for our package named ddbot_description. Next, we will create three new folders in our package.

```bash
cd ddbot_description/
mkdir launch urdf rviz
```

## Robot Model

The robot model consists of link elements, joint elements, and gazebo plugins. A link element is a rigid component. Links are attached to other links via joints. The joint elements specify the relative motion between links. Typically, joints allow for rotation or translation.

The common properties specified for links and joints are inertial, collision, and visual. Inertial and collision properties enable physics simulation. The visual properties control the robot's appearance.

ROS uses Unified Robot Description Format (URDF) files to specify a robot's properties. URDF supports XML ans xacro (XML macro) languages.

We will build our robot model, step by step, first specifying its visual properties.</br></br>

### Step 1 - Base Link Visual Model

Use a text editor to create a file named ddbot.urdf in the urdf directory. Add the code below to the file and save it.

```xml
<?xml version="1.0"?>
<robot name="ddbot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.07"/>
      </geometry>
    </visual>
  </link>
</robot>
```

Translating the XML into English, this is a robot with the name ddbot. It contains a link named base_link whose visual geometry is a rectangular box 0.5 meters wide, 0.3 meters high, and 0.07 meters long. The size attribute contains the three side lengths of the box. The origin of the box is in its center.

Create a file named display.launch in the src/ddbot_description/launch directory. Add the code shown below and save it.

```xml
<?xml version="1.0"?>
<launch>

  <arg name="model" default="$(find ddbot_description)/urdf/ddbot.urdf"/>
  <arg name="gui" default="false" />
  <arg name="rvizconfig" default="$(find ddbot_description)/rviz/urdf.rviz" />

  <param name="robot_description" command="$(find xacro)/xacro $(arg model)" />
  <node if="$(arg gui)" name="joint_state_publisher" pkg="joint_state_publisher_gui" type="joint_state_publisher_gui" />
  <node unless="$(arg gui)" name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />

  <node name="rviz" pkg="rviz" type="rviz" args="-d $(arg rvizconfig)" required="true" />
</launch>
```

</br></br>

Add the file named urdf.rviz to the src/ddbot_description/rviz directory. We are finally ready to view our robot model in rviz. In the terminal, navigate to the ddbot_ws folder and run catkin_make.  

```bash
cd ~/ddbot_ws
catkin_make
```

Two new directories are added to your ddbot_ws folder, build and devel. Run the source command below to overlay the workspace.

```bash
source devel/setup.bash
```

Now, we use the launch file to start rviz and see our robot model.

```bash
roslaunch ddbot_description display.launch
```

RViz should display the following:

![rviz base link](./images/rviz_base_link.png "rviz base_link")</br></br>

Note:
   - The fixed frame is the transform frame where the center of the grid is located. Here, it's a frame defined by our one link, base_link.
   - The visual element (the box) has its origin at the center of its geometry as a default. Look carefully to see that half the box is below the grid.

Close rviz.

## Add Color to Model

Update the ddbot.urdf file to add material tags. Blue and white material tags are added at the top of the file to define color. The color value range is [0,1]. rgba stands for red, green, blue, alpha. Alpha is the transparency.

Add the blue material to the base_link to change the color to blue. The updated ddbot.urdf file, as shown below.

```xml
<?xml version="1.0"?>
<robot name="ddbot">
  <material name="blue">
    <color rgba="0 0 0.8 1"/>
  </material>

  <material name="white">
    <color rgba="1 1 1 1"/>
  </material>


  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.07"/>
      </geometry>
      <material name="blue"/>
    </visual>
    
  </link>
</robot>
```

Launch the updated description to see that the base link box is now blue.

```bash
roslaunch ddbot_description display.launch
``` 

</br></br>

![blue base link](./images/blue_base_link.png "blue base_link")</br></br>


## Add Wheel Links and Joints

We add two links to the description, for the right and left wheels. The wheel visual geometry is a cylinder. The

```xml
<link name="right_wheel">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.3"/>
      </geometry>
      <material name="white"/>
    </visual>
  </link>

  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.3"/>
      </geometry>
      <material name="white"/>
    </visual>
  </link>
```

We also need to specify where the wheels attach to the base_link. Joint elements are used to create the attachment. A joint can be flexible or inflexible. In this example

### Model Summary

- link elements
   - rectangular chassis
   - two cylindrical drive wheels (left and right)
   - one caster wheel
- joint elements
   - define relative motion of wheels with respect to the chassis
   - specify values for damping and friction
- gazebo plugins
   - model plugin: differential drive
      - accepts velocity commands and publishes odometry information
   - sensor plugin: hoyuko laser scanner
</br></br>

![robot model](./images/lesson01/robot_model.png "robot model")</br></br>

### Visualize Robot in rviz

rviz is a 3D visualization software tool for robots, sensors, and algorithms. It provides a view of your robot model, captures sensor information from robot sensors, and replays captured data. It can display data from camera, lasers, from 3D and 2D devices including pictures and point clouds. rviz enables you to see the robot’s perception of its world (real or simulated).</br></br>

Open a terminal and navigate to the simulation workspace directory simulation_ws. Anytime we open a new terminal session or rebuild our code, we need to use the source command and setup.bash script to configure the environment. We use the roslaunch tool to start rviz using the launch file rviz.launch.

```bash
cd ros_gazebo_two_wheel_robot/simulation_ws
source ./devel/setup.bash
roslaunch m2wr_description rviz.launch
```

rviz will launch with an empty grid space and a Fixed Frame global status error, as shown below. </br></br>

![rviz global error](./images/lesson01/global_status_error.png "global status error")</br></br>

Change the Fixed Frame map selection to link_chassis. Click on map, choosing link_chassis from the dropdown.</br></br>

![fixed frame link chassis](./images/lesson01/fixed_frame_link_chassis.png "fixed frame link chassis")</br></br>

Next, click the Add button and choose RobotModel. </br></br>

![add robot model](./images/lesson01/add_robot_model.png "add robot model")</br></br>

Now you see the two-wheel differential drive robot description has been added to the display.</br></br>

![robot model added](./images/lesson01/robot_model_added.png "robot model added")</br></br>

We will return to rviz in another lesson. For now, close rviz. You may save your file so that you don't have to reset the fixed frame and add the robot model the next time rviz is launched.</br></br>

## Additional Study

Read and work through the following tutorials to gain an understanding of how the robot model is created.

1. Gazebo, [Make a Mobile Robot tutorial](http://gazebosim.org/tutorials/?tut=build_robot), "demonstrates Gazebo's basic model management, and exercises familiarity with basic model representation inside the model database by taking the user through the process of creating a two wheeled mobile robot that uses a differential drive mechanism for movement." The tutorial adds one component at a time, allowing for experimentation and instantaneous visualization of the effects of changes.

2. [The Construct](https://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/) video provides a detailed explanation of the robot model development and source code.

3. [Chapter 2](https://kiranpalla.com/autonomous-navigation-ros-differential-drive-robot-simulation/describing-ros-robot-with-urdf/) of Kiran Palla's free ebook, Autonomous Navigation of ROS Robot: Differential Drive Robot Simulation, provides another example of creating a ROS package for the same differential drive robot form. </br></br>


4. ROS urdf tutorials, http://wiki.ros.org/urdf/Tutorials