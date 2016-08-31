# load-share-estimation

This package estimates the proportion of an object's load supported by a robot,
using readings from a force/torque sensor mounted on the robot wrist, and the
state of the robot end-effector (including the position and acceleration).

The only necessary information about the object is the object's mass.

We require a few transforms to be defined, namely a world frame, a robot root
frame, and a frame on the force/torque sensor.

## Inputs

Masses of


he experimentally-determined mass of the f/t sensor plate
To find it again, calibrate with the plate down, flip the sensor so the plate is up, and solve for:
-2 * m * g = F_z


Robot end-effector acceleration

Force/Torque data


## Outputs

The node outputs information on several topic:

 - load share (float64): The proportion of the object's mass supported by the
   robot, in the rangefrom 0 (no support) to 1.0 (full support).

 - internal wrench (WrenchStamped): Internal forces on the object that are
  counteracting forces between the robot and the other party (these are
  unecessary to accomplish a transfer).

 - dynamics load share (float64): The proportion of the load share which is due
   to object dynamics (i.e. robot motion) and not the object transfer.

In addition, we publis some debugging information on sub-topics:

 - current force (WrenchStamped): this is the currently measured force,
   expressed in the world frame.

 - object dynamics, expected and observed (WrenchStamped): Forces expected and
   measured that are due to the motion of the object (dynamics).


## Calibration



## Other parameters

 - force/torque delay: In some cases, a filter to compute the robot acceleration
   induces a delay in the acceleration measurement. We use a ROS
   `message_filters::TimeSequencer` to add a constant time offset to the
   incoming f/t messages.

 - force/torque filtering: We filter the force and torque measurement using an
   exponential smoother, you can change the filtering parameter (for example,
   this might be important if you change the running frequency of the estimation
   loop).

 - load share filtering: We filter the computed load share in the exact same
   way, you can change the exponential smoother parameter.


Random notes


SG filter for acceleration:
 - kuka-lwr-ros/kuka_lwr/lwr_controllers/include/lwr_controllers/kuka_joint_state_controller.h
 - kuka-lwr-ros/kuka_lwr/lwr_controllers/src/kuka_joint_state_controller.cpp






Calibration: zeros out the f/t readings.
