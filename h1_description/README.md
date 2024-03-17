# H1 Description

## Overview

This directory contains URDF descriptions of Unitree's H1 humanoid robot. The original description files can be found [here](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/h1_description).

Note that I've not included the version of the humanoid with dextrous hands.

The models under `urdf/` can be used in a regular ROS setting (Although it's not something I've really tested).

The models under `drake_urdf/` are the versions of the model that I modified for use with Drake. These ones have Drake specific extension tags added to them.

## Additional modifications

I've left the visual meshes untouched but I've updated the collision meshes to use simple convex geometries (boxes, cylinders and spheres) as their original geometries were incorrect.

## Visuals

### Visual model

### Collision geometry

### Inertia representations


