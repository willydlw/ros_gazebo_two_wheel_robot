# Adding Physical Properties

With the additional physical properties of mass and inertia, our robot will be ready to be launched in the Gazebo simulator. These properties are needed by Gazebo's physics engine. Specifically, every link element that is being simulated needs an inertial tag.

The two sub-elements of the inertial element we will use are as follows:

- mass: This is the weight defined in kilograms.

- inertia: This frame is a 3 x 3 rotational inertia matrix.  

| ixx | ixy | ixz |
| --- | --- | --- |
| ixy | iyy | iyz |
| ixz | iyz | izz |

</br>

The rotational inertia matrix is symmetrical, so it can be represented by only six elements: ixx, ixy, ixz, iyy, iyz, izz.  

Wikipedia's list of moment of inertia tensors (https://en.wikipedia.org/wiki/List_of_moments_of_inertia) provides the equations for the inertia of simple geometric primitives, such as a cylinder, box, and sphere. We use these equations to compute the inertia values for the model's chassis, caster, and wheels.

Do not use inertia elements of zero (or almost zero) because real-time controllers can cause the robot model to collapse without warning, and all links will appear with their origins coinciding with the world origin.

The file dd_robot6.urdf file contains the additional XML code shown below.

```xml
<?xml version='1.0'?>
<robot name="dd_robot">

  <!-- Base Link -->
  ...
    <inertial>
      <mass value="5"/>
      <inertia ixx="0.13" ixy="0.0" ixz="0.0"
               iyy="0.21" iyz="0.0" izz="0.13"/>
    </inertial>

    <!-- Caster -->
    ...
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0"
               iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Right Wheel -->
  ...
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01"  ixy="0.0" ixz="0.0"
               iyy="0.005" iyz="0.0" izz="0.005"/>
    </inertial>
  ...
  <!-- Left Wheel -->
  ...
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0"
               iyy="0.005" iyz="0.0" izz="0.005"/>
    </inertial>
    ...
</robot>
```

