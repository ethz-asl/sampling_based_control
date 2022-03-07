# mppi_manipulation_royalpanda

This package provides support for deployment of the sampling based controller on the Royalpanda platform.
The `mppi_manipulation` controller interface is wrapped in the a ROS controller [here](./include/mppi_manipulation_royalpanda/controller.h).

## Important 
Berfore reading further make sure you have familiarity with the ROS control framework: http://wiki.ros.org/ros_control. It would be too hard to explain it here and we would definetely duplicate explanations already provided in the Wiki.  

## Simulation support
In order to provide a better debug experience, this package also contains a simulator which offers the same exact interfaces as on the real  robot. In this way, we can test the controller functionality and implementation without the need for the real physical robot.
The simulation is implemented [here](./include/mppi_manipulation_royalpanda/simulation.h). It offers the following interfaces:
- `EffortJointInterface`: used to write torque commands to the Panda robot
- `FrankaModelInterface`: used as a convenience method to access dynamic information (coriolis, gravity, ...)
- `FrankaStateInterface`: used to access the franka state (joint position, velocity, effort, wrench, ...)
- `VelocityJointInterface`: this is where we have the main difference from sim to real. On the real robot, the base is controlled through a separate ros topic receiving twist commands. As in simulation we can directly control the base, we offer a `VelocityJointInterface` to allow for direct write of velocity commands. If no such interface is found, we fallback to sending twist commands over topic (we auto-detect that we are not in sim).

## State observer
The controller state is composed of:
- the _arm state_ : position, velocity and effort of all arm joints and the two fingers mounted on the gripper
- the _base state_ : position, velocity and effort associated with the 3 degress of freedom of the base
- a _contact flag_ : a flag indicating contact with the environment
- the _object state_ : position and velocity of the 1-DOF articulated object

### Retrieving the arm state
In order to get the arm state, we can exploit the fact that the controller has access to the `FrankaStateInterface`.  Additionally this state is also available as a `JointState` message at the topic `franka_state_controller/joint_states`. 

In simulation this is published by the simulation node itself, while on the real robot, this is published by the real robot state controller.

Note that this only provides the state of the arm __excluding the fingers__. When we deploy the controller, we also start the _gripper controller_ (check this [launch file](./launch/robot.launch)). This will publish the finger state under the topic `franka_gripper/joint_states`. Again, we made sure that the simulation also sends the finger state on this topic so that all the information available when running the real robot is available at test time. 

### Retrieving the base state
The ridgeback (or simulation) publishes odometry messages on the `/ridgeback_velocity_controller/odom` topic. We extract only the twist from this message and instead rely on a motion tracking system for the measurement of the base position (as odometry will inevitably drift over time). 
The motion tracking system used in our experiments publishes odom messages which we use in the state observer to extract the position. When running the simulation this is published by the simulation node. 

### Retrieving the object state
This is quite tricky, embrace yourself. 

The object is actually described by its 6-DOF pose and the state of the articulation (1-DOF). The controller does not need the full pose since it already knows (through the URDF) where the object is located in space. While in simulation we use __the same URDF in simulation and in the controller__ (and therefore it is impossible that they disagree on the object location), in the real world the URDF might not match the real pose of the object. We describe here the modus operandi for our experiment (maybe not optimal but it is at least something).

At the beginning of each experiment we run the state observer node (not introduced yet but bear with me) that __assumes that the object is initially closed__. Thanks to this assumption and the known URDF (which is assumed to be accurate), the state observer recostruct the kinematic chain and determines where the object is located in space. The static transform is then published to ROS. One can obtain the transform either reading the printout to the terminal or with `rosrun tf tf_echo world <base_link>` and replace this transformation in the URDF. Then we stop the node and rerun everything again with the __tuned URDF__. Note that any small motion of the object will require to perform another "calibration" round. 

Ok! Now that we know the object pose and this is encoded the URDF, how do we get the position and velocity of the joint? We have some markers placed on the moving part of the object. Afterward, we make sure that the pose published by the motion tracking system corresponds to a link in the URDF. In our experiment the published pose was the pose of the handle. In this way we could extract the rotation angle and joint rotation speed from two subsequent poses received from the Motion Tracking System.

__Note__
This is quite cumbersome and suboptimal. An alternative method, could be to refactor the state observer node such that it subscribes to whatever pose from the motion tracking system (not necessarily equivalent to a "twin" frame in the URDF) and while moving the object it estimates the rotation axis (a simple circle fitting). From the axis location and the urdf, one can easily retrieve the joint position and pose of the object (with assumption of initially closed object). Feel free to implement this or a better method!

### Putting all together
Finally the _state observer node_ to the rescue! It is simply subscribing to all these source of information and asynchronously updating a compound state. It uses also other libraries (`orocos_kdl`) to read the URDF and use it for the calculation explained in the previous section. 

The full state is then published over ros as a `manipulation_msgs::State` message at `/observer/state`. The controller, subscribes to this message and used the information to update the internal controller state. Conversion from/to ROS to/from Eigen vectors is supported through a set of functions defined in the `manipulation_msgs` package. 
Below a schematics of the system showing that indeed sim and real share the same information:

![alt text](./docs/schematics.png)


### Running the simulation
The simulation launches two nodes:
- A `state observer` node,
- a `simulation` node.
Run it with:
```
roslaunch mppi_manipulation_royalpanda sim.launch
```

## Running on the robot
When deploying on the robot, the robot is in charge of internally loading the controller and exposing all the necessary interfaces. As these are the same we tested in sim we do not have to change the controller (except for handling the velocity twist commands as explained in the previous section). 

### Motion Tracking System
We use the VICON Motion Tracking System. This provides a ros bridge for publishing marker poses (how we use them in the previous section). These instructions need to be adapted for a new tracking system in case VICON is not used. Refer to each launch file, for the relevant parameters. 

### What to launch
On the robot computer (the one routed to panda and on the same ROS network as the ridgeback ROS master)
```
roslaunch mppi_manipulation_royalpanda demo.launch
```
On the operator computer (on the same ROS network as the ridgeback ROS master). The operator launch uses `plotjuggler_ros`. It uses layout files that can be saved to avoid refactoring the UI every time you launch the node (some probably outdated are saved in `./conig/pj/*`). Make sure you have this dependency installed if you want to use it. 
```
roslaunch mppi_manipulation_royalapanda operator_pc.launch
```
