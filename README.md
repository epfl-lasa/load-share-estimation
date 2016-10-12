# Load Share Estimation
[![Build Status](https://travis-ci.com/epfl-lasa/load-share-estimation.svg?token=BqUQb763tsVV4QyzLgBy&branch=master)](https://travis-ci.com/epfl-lasa/load-share-estimation)

This ROS package is used to compute the load share of an object being supported by a robot and a third party (such as a person).
The **load share** is the proportion of the object's weight supported by the robot, and varies during cooperative tasks such as manipulation or handovers.
For example during a handover, the load share smoothly goes from 1.0 (the robot is supporting the object) to 0.0 (the human is supporting the object).
Properly estimating this load share is key to enabling smooth human-robot handovers.

Some features of this package are:

 - Works in any end-effector configuration: up, down, sideways, etc...
 - Accounts for object dynamics (i.e. forces due to acceleration).
 - Input-light: requires only force/torque sensor readings and end-effector acceleration.
 - The only information necessary about the object is its *mass*.
 - Only requires knowing a few masses in the system: the object, the tool (hand/gripper/etc..), and the force/torque sensor plate.
 - Highly configurable: change topics, filtering parameters, masses, etc... through a simple yaml configuration file.

The rest of this README explains in detail the necessary inputs and how to configure the package.
We also provide a complete example using recorded data.

This package is an open-source implementation of the load share estimation module in the following paper:

> A Human-Inspired Controller for Fluid Human-Robot Handovers. International Conference on Humanoid Robots, 2016. Jose Medina, Felix Duvallet, Murali Karnam, and Aude Billard.

If you use this package in your research, please cite the above paper.

## Running a simple example.

## Inputs and required components

The components we reason about are:
 - The robot end-effector
 - The force/torque sensor (mounted on the end-effector)
 - The tool: what is holding the object (mounted on the other side of the force/torque sensor), it could be a hand, a gripper, tray, etc...
 - The object: what we are transferring.

We must know the following component masses:

 - The mass of the object (in kg)
 - The mass of the tool (in kg)
 - The mass of the force/torque sensor plate (in kg), which is the part that's not bolted on the arm.

Note: the f/t sensor plate mass is a little tricky to estimate.
We determined it experimentally by zero-ing the sensor with the plate hanging
freely down, then flipping the sensor so the plate is up (all forces should be
along the z-axis).
Then, the mass of the plate (m_plate) will be given by the force in the z-axis (F_z):

    -2 * m_plate * g = F_z

The node computes the **wrenches expressed in the world frame**.
For this, we need access to the following pieces of information:

  - The force/torque sensor data.
  - The end-effector acceleration.
  - The sensor's pose (as a `tf`).
  - The robot root's pose (as a `tf) in the world frame.

The acceleration (along with information about the hand/object) enables us to compute the effect of the robot dynamic on the sensed forces (to remove the forces due to acceleration).
The transforms enable us to express the forces in the world frame.

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

It is very important to calibrate the f/t sensor before running the node.
This calibration should occur without the object in the hand.

At calibration time, we store the robot end-effector orientation as a ROS
parameter, so that we can recover the actual forces during operation.

The calibration procedure should 'zero out' the force/torque sensor. In other
words, after calibration the force and torque measurements should be
approximately zero.


## Other parameters

 - force/torque delay: In some cases, a filter to compute the robot acceleration
   induces a delay in the acceleration measurement. We use a ROS
   `message_filters::TimeSequencer` to add a delay the incoming f/t messages
   by a constant time offset.

 - force/torque filtering: We filter the force and torque measurement using an
   exponential smoother, you can change the filtering parameter (for example,
   this might be important if you change the running frequency of the estimation
   loop).

 - load share filtering: We filter the computed load share in the exact same
   way, you can change the exponential smoother parameter.
