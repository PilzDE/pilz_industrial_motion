# Integration test setup for ur3
- Use [ros-industrial/universal_robot](https://github.com/ros-industrial/universal_robot) repository for `melodic-devel` support of universal robot ur3.
- To use the launch files of the ur3 provided by [ros-industrial/universal_robot](https://github.com/ros-industrial/universal_robot) repository, the end-effector group `endeffector` must be commented out in `ur3_moveit_config/config/ur3.srdf`.  
  
**Please note:**  
It necessary to comment out the `endeffector` group because the `pilz_command_planner` states that the limits for the joints in the `endeffector` group are not set. (The `endeffector` group in the default `ur3.srdf` file actually consists of no joints, still the `pilz_command_planner` gives the before mentioned error message.)

