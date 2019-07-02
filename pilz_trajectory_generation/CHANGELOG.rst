^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pilz_trajectory_generation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.8 (2019-07-02)
------------------
* Commenting unstable test with franka emika panda

0.3.7 (2019-05-09)
------------------
* fixed an error that led to trajectories not strictly increasing in time
* update dependencies of trajectory_generation
* fix CIRC path generator and increase test coverage
* adopt strictest limits in ptp planner (refactor JointLimitsContainer and TrajectoryGeneratorPTP)
* Enable gripper commands inside a sequence
* Contributors: Pilz GmbH and Co. KG

0.3.6 (2019-02-26)
------------------
* refactor the testdataloader

0.3.5 (2019-02-06)
------------------
* Increase line coverage for blending to 100%

0.3.4 (2019-02-05)
------------------
* refactor determining the trajectory alignment in the blend implementation
* extend and refactor unittest of blender_transition_window
* add planning group check to blender_transition_window

0.3.3 (2019-01-25)
------------------
* add more details to blend algorithm description
* change handling of empty sequences in capabilities to be non-erroneous
* rename command_planner -> pilz_command_planner

0.3.2 (2019-01-18)
------------------
* use pilz_testutils package for blend test
* use collision-aware ik calculation
* Contributors: Pilz GmbH and Co. KG

0.3.1 (2018-12-17)
------------------

0.3.0 (2018-11-28)
------------------
* add append method for avoiding duplicate points in robot_trajectory trajectories
* Relax the precondition on trajectory generators from v_start==0 to |v_start| < 1e-10 to gain robustness
* Set last point of generated trajectories to have vel=acc=0 to match the first point.
* add sequence action and service capabilities to concatenate multiple requests
* Contributors: Pilz GmbH and Co. KG

0.1.1 (2018-09-25)
------------------
* port to melodic
* drop unused dependencies
* Contributors: Pilz GmbH and Co. KG

0.1.0 (2018-09-14)
------------------
* Created trajectory generation package with ptp, lin, circ and blend planner
* Contributors: Pilz GmbH and Co. KG
