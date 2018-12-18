^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pilz_trajectory_generation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2018-12-18)
------------------
* Use Eigen::Isometry3d to keep up with the recent changes in moveit
* Contributors: Chris Lalancette

0.3.1 (2018-12-17)
------------------

0.3.0 (2018-11-28)
------------------
* add append method for avoiding duplicate points in robot_trajectory trajectories
* Relax the precondition on trajectory generators from v_start==0 to |v_start| < 1e-10 to gain robustness
* Set last point of generated trajectories to have vel=acc=0 to match the first point.
* add sequence action and service capabilities to concatenate multiple requests
* Contributors: Pilz GmbH and Co. KG

0.2.2 (2018-09-26)
------------------

0.2.1 (2018-09-25)
------------------

0.1.1 (2018-09-25)
------------------
* port to melodic
* drop unused dependencies
* Contributors: Pilz GmbH and Co. KG

0.2.0 (2018-09-14)
------------------
* Changes for melodic
* Contributors: Pilz GmbH and Co. KG

0.1.0 (2018-09-14)
------------------
* Created trajectory generation package with ptp, lin, circ and blend planner
* Contributors: Pilz GmbH and Co. KG
