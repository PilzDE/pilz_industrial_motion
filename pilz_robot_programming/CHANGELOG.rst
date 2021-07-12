^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pilz_robot_programming
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.13 (2021-07-12)
-------------------
* Merge our trajectory_generation as moveit planner
  * update all references to the planner
  * move sequence related pilz msgs to moveit
  * change blend radius in test data to not cause blend radius to large error
* Prevent misunderstandings in acceptance-tests
* Contributors: Pilz GmbH and Co. KG

0.4.12 (2020-11-24)
-------------------
* Adapt to generalized test-utils
* Add missing test-depend on prbt_hardware_support
* Contributors: Pilz GmbH and Co. KG

0.4.11 (2020-07-16)
-------------------
* Add Attribute based equivalence for commands.
* Add feature to store points.
* Add PoseStamped and tuple goals.
* Add get_current_pose_stamped to robot api
* Replace tf by tf2 in pilz_robot_programming.
* Rename _BaseCmd -> BaseCmd.
* Remove outdated add_python_coverage() function.
* Remove outdated/superfluous documents.
* Remove static version from doc.
* Remove unused import of prbt_hardware_support.
* Fix acceptance tests.
* Fix segfault on shutdown.
* Fix python 3 compatibility issues.

0.4.10 (2019-12-04)
-------------------
* Adapt to new brake test srv definitions in pilz_msgs
* Contributors: Pilz GmbH and Co. KG

0.4.9 (2019-11-28)
------------------
* Import speed override srv from pilz_msgs
* Contributors: Pilz GmbH and Co. KG

0.4.8 (2019-11-22)
------------------
* Drop unused variables in python api (#162)
* Override speed of motions
* Contributors: Pilz GmbH and Co. KG

0.4.7 (2019-09-10)
------------------

0.4.6 (2019-09-04)
------------------

0.4.5 (2019-09-03)
------------------
* fix PEP issues
* Contributors: Pilz GmbH and Co. KG

0.4.4 (2019-06-19)
------------------
* Add python api methods for brake tests

0.4.3 (2019-04-08)
------------------

0.4.2 (2019-03-13)
------------------

0.4.1 (2019-02-27)
------------------
* Minor fixes
* Contributors: Pilz GmbH and Co. KG

0.3.5 (2019-02-06)
------------------

0.3.4 (2019-02-05)
------------------
* enable Robot instantiation after a program got killed; add corresponding test
* apply renaming command_planner -> pilz_command_planner
* Contributors: Pilz GmbH and Co. KG

0.4.0 (2018-12-18)
------------------
* Release Python-API from kinetic version 0.3.1

0.3.1 (2018-12-17)
------------------
* Add RobotMotionObserver in testutils
* Contributors: Pilz GmbH and Co. KG

0.3.0 (2018-11-28)
------------------
* Release Python-API
* Contributors: Pilz GmbH and Co. KG
