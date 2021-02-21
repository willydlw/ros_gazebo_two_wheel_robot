# ros_gazebo_two_wheel_robot
2 wheeled robot simulation using ros and gazebo



### Test Environment

- ROS: Melodic
- Ubuntu: 18.04
- Gazebo: 9.0.0
- Python: 3.6.9


#### Simulation Workspace Folder
    .
    └── simulation_ws           # Simulation Workspace
        ├── src                 # Source files 
        │   ├── robot_description       # Robot definition directory
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
    
### Setup

After downloading the files, navigate to the project folder
```
cd ros_gazebo_two_wheel_robot
```
Navigate to Catkin Workspace and run `catkin_make`
```
cd catkin_ws
catkin_make
```
Return to the project folder 
```
cd ..
```
Navigate to Simulation Workspace and run `catkin_make`
```
cd simulation_ws
catkin_make
```
## Lessons

### Lesson 1 - Visualizing the robot with rviz

