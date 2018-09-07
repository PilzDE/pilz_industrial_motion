# Package: pilz_industrial_motion
Meta package containing the current results of the
[ROSIN FTP: Industrial trajectory generation for MoveIt!](http://rosin-project.eu/ftp/industrial-trajectory-generation).
The FTP is implemented by Pilz GmbH & Co. KG.

![PilzIndustrialMotion](http://rosin-project.eu/wp-content/uploads/prbt_Service_Robotics_Modules_plus_Operator-1024x683.jpg)

## Build Status

| Kinetic | Melodic |
| --------|-------- |
| [![Build Status](https://travis-ci.org/PilzDE/pilz_robots.svg?branch=kinetic-devel)](https://travis-ci.org/PilzDE/pilz_industrial_motion) | [![Build Status](https://travis-ci.org/PilzDE/pilz_robots.svg?branch=melodic-devel)](https://travis-ci.org/PilzDE/pilz_industrial_motion) |


## Package: pilz_trajectory_generation
Provides the generators to create LIN, PTP and CIRC trajectories. It uses a plugin structure which allows to the
definition of custom commands.

## Package: pilz_extensions
Minor extensions of existing packages needed for the trajectory_generation.

## Package: pilz_testutils
Contains helper classes and functions that are used by the tests



