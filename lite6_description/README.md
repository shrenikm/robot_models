# Lite6 Description

## Overview

This directory contains URDF descriptions of Ufactory's lite6 manipulator. The original description can be found [here](https://github.com/xArm-Developer/xarm_ros/tree/master).

I've parsed them into quite a few files:

* URDF models of just the grippers (both parallel and vacuum)
* URDF models of the manipulator without grippers
* URDF models fo the manipualtor with grippers attached (both parallel and vacuum)

The models under `urdf/` can be used in a regular ROS setting (Although it's not something I've really tested). Note that these are post xacro compilation. I've also taken the liberty of cleaning some stuff up.

The models under `drake_urdf/` are the versions of the model that I modified for use with Drake. These ones have the ROS/Gazebo tags stripped and Drake specific extension tags added to them.

## Additional modifications

* The original files come with a non-actuated parallel gripper. I've modified the mesh files here to remove the non-actuated gripper part. The URDF now has geometry that reflect the prongs of the gripper, and is actuated by a prismatic joint
* Two models of the parallel grippers are provided: `normal` and `reversed` as named in their user manual that indicates the orientation of the prongs when screwed on. In `normal` mode, the gripper prongs make contact when closed but only open a little. In `reversed` mode, the gripper prongs can open wider, but don't make contact when closed. The total translational distance is the same in both cases. I've tried to make these as accurate to the real gripper as possible
* I've left the visual meshes untouched but I've updated the collision meshes to use simple convex geometries (boxes, cylinders, ellipsoids) as their original meshes would cause issues during simulation. The original collision meshes are still provided
* I've also left in the FreeCAD files used to generate the collision meshes

## Visuals
