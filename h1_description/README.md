# H1 Description

## Overview

This directory contains URDF descriptions of Unitree's H1 humanoid robot. The original description files can be found [here](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/h1_description).

Note that I've not included the version of the humanoid with dextrous hands.

The models under `urdf/` can be used in a regular ROS setting (Although it's not something I've really tested).

The models under `drake_urdf/` are the versions of the model that I modified for use with Drake. These ones have Drake specific extension tags added to them.

## Additional modifications

I've left the visual meshes untouched but I've updated the collision geometries as the original ones were incorrect.

## Visuals

### Visual model

![h1_visual](https://github.com/shrenikm/robot_models/assets/11331850/6c726b4c-2500-476e-8a8f-38fbf9cb0d9b)

### Collision geometry

![h1_collision](https://github.com/shrenikm/robot_models/assets/11331850/d2465fcb-4429-4db7-bb5a-39fb70b18320)

### Inertia representations

![h1_inertia](https://github.com/shrenikm/robot_models/assets/11331850/50412bb6-160e-4fbd-8e76-418a54b463e4)




