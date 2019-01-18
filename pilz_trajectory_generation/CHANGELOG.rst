^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pilz_trajectory_generation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
