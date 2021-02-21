# Lesson 1 - Visualizing the robot with rviz

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