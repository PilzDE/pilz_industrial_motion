# Package: pilz_industrial_motion
Meta package containing the current results of the
[ROSIN FTP: Industrial trajectory generation for MoveIt!](http://rosin-project.eu/ftp/industrial-trajectory-generation).
The FTP is implemented by Pilz GmbH & Co. KG.

![PilzIndustrialMotion](http://rosin-project.eu/wp-content/uploads/prbt_Service_Robotics_Modules_plus_Operator-1024x683.jpg)

## Build Status

|   | Kinetic | Melodic | Noetic |
| ----| --------|-------- |-------- |
| Travis/GitHub CI  | [![Build Status](https://travis-ci.org/PilzDE/pilz_industrial_motion.svg?branch=kinetic-devel)](https://travis-ci.org/PilzDE/pilz_industrial_motion) | [![CI](https://github.com/PilzDE/pilz_industrial_motion/workflows/CI/badge.svg?branch=melodic-devel&event=push)](https://github.com/PilzDE/pilz_industrial_motion/actions?query=event%3Apush+workflow%3ACI+branch%3Amelodic-devel) | [![CI-noetic](https://github.com/PilzDE/pilz_industrial_motion/workflows/CI-noetic/badge.svg?event=push)](https://github.com/PilzDE/pilz_industrial_motion/actions?query=event%3Apush+workflow%3ACI-noetic+branch%3Anoetic-devel) |
| Buildfarm src | [![buildfarm](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__pilz_industrial_motion__ubuntu_xenial__source)](http://build.ros.org/view/Ksrc_uX/job/Ksrc_uX__pilz_industrial_motion__ubuntu_xenial__source/) | [![buildfarm](http://build.ros.org/buildStatus/icon?job=Msrc_uB__pilz_industrial_motion__ubuntu_bionic__source)](http://build.ros.org/view/Msrc_uB/job/Msrc_uB__pilz_industrial_motion__ubuntu_bionic__source/) | [![buildfarm](http://build.ros.org/buildStatus/icon?job=Nsrc_uF__pilz_industrial_motion__ubuntu_focal__source)](http://build.ros.org/view/Nsrc_uF/job/Nsrc_uF__pilz_industrial_motion__ubuntu_focal__source/) |
| Buildfarm bin | [![buildfarm](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__pilz_industrial_motion__ubuntu_xenial_amd64__binary)](http://build.ros.org/view/Kbin_uX64/job/Kbin_uX64__pilz_industrial_motion__ubuntu_xenial_amd64__binary/) | [![buildfarm](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__pilz_industrial_motion__ubuntu_bionic_amd64__binary)](http://build.ros.org/view/Mbin_uB64/job/Mbin_uB64__pilz_industrial_motion__ubuntu_bionic_amd64__binary/)| [![buildfarm](http://build.ros.org/buildStatus/icon?job=Nbin_uF64__pilz_industrial_motion__ubuntu_focal_amd64__binary)](http://build.ros.org/view/Nbin_uF64/job/Nbin_uF64__pilz_industrial_motion__ubuntu_focal_amd64__binary/) |

### Note to Developers:
`noetic-devel` is considered to be the active development branch.
Relevant changes are cherry-picked into `melodic-devel` or `kinetic-devel` on a case-by-case basis.

## Package: pilz_trajectory_generation
This package used to be found here but is now moved to [moveit](https://moveit.ros.org/documentation/planners/)

## Package: pilz\_robot\_programming
pilz\_robot\_programming provides a python API for an intuitive programming of a MoveIt! enabled robot. For details please refer to the [ Pilz Robot Programming Documentation ](pilz_robot_programming/Readme.rst).

***
<!--
    ROSIN acknowledgement from the ROSIN press kit
    @ https://github.com/rosin-project/press_kit
-->

<a href="http://rosin-project.eu">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png"
       alt="rosin_logo" height="60" >
</a>

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.
More information: <a href="http://rosin-project.eu">rosin-project.eu</a>

<img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg"
     alt="eu_flag" height="45" align="left" >

This project has received funding from the European Union’s Horizon 2020
research and innovation programme under grant agreement no. 732287.
