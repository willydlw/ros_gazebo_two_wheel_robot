# Lesson 1 - Visualizing the robot with rviz

This lesson illustrates how to add our robot model to rviz.</br></br>

## Robot Model

The robot description model files are found in the simulation workspace directory. The URDF (Unified Robot Description Format) and XACRO files are the same as those developed by the The Construct, Exploring ROS Using A 2 Wheeled Robot, lessons 1 - 3. 

Watch the lesson videos, https://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/, for a detailed explanation of the robot model development and source code.

Additionally, the Gazebo, Make a Mobile Robot tutorial, http://gazebosim.org/tutorials/?tut=build_robot, "demonstrates Gazebo's basic model management, and exercises familiarity with basic model representation inside the model database by taking the user through the process of creating a two wheeled mobile robot that uses a differential drive mechanism for movement." I enjoyed this tutorial as it adds one component at a time and immediately shows the model. This allows for experimentation and instantaneous visualization of the effects of changes.</br></br>

### Terminal

Open a terminal and type the following.

```
cd simulation_ws
source ./devel/setup.bash
roslaunch robot_description rviz.launch
```

Rviz will launch with a Fixed Frame global status error. </br></br>

![rviz global error](./images/lesson01/global_status_error.png "global status error")</br></br>

Change the Fixed Frame map selection to link_chassis. Click on map, choosing link_chassis from the dropdown.</br></br>

![fixed frame link chassis](./images/lesson01/fixed_frame_link_chassis.png "fixed frame link chassis")</br></br>

Next, click the Add button and choose RobotModel. </br></br>

![add robot model](./images/lesson01/add_robot_model.png "add robot model")</br></br>


Now you see the two-wheel differential drive robot description has been added to the display.</br></br>

![robot model added](./images/lesson01/robot_model_added.png "robot model added")</br></br>

We will return to rviz in another lesson. For now, close rviz. You may save your file so that you don't have to reset the fixed frame and add the robot model the next time rviz is launched.