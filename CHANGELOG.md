# Changelog

All notable changes to this project are listed here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and versions follow
[Semantic Versioning](https://semver.org/).

## [1.0.0] - 2026-11-26

A ROS Melodic and Gazebo simulation of a 5-DoF robot arm. It detects objects with an RGB-D camera, estimates their 6-DoF pose and grasps them with MoveIt.

### Added

- The original 2020 work covers the `five_dof_manipulator` package, with the arm's URDF and meshes, Gazebo worlds, controllers, perception launch files, planners (`jointplan.py`, `demo_obj.py`) and demos.
- `robot_moveit_config`, the MoveIt configuration, and `robot_arm_ikfast_plugin`, the IKFast kinematics plugin for the arm.
- `package.xml` dependency lists, an MIT license for `five_dof_manipulator` and the pictures, and a `.gitignore`.
- A README with a system diagram, an interface table, numbered setup steps, usage, extension points and limitations.

### Changed

- Package paths in the launch files and the URDF point at `five_dof_manipulator`, where they pointed at `six_dof` and at a home directory before.
- The IKFast update script finds its own package with `rospack find`, where it pointed at a home directory before.
- `catesian.py` is now `cartesian.py`.

### Removed

- The unused `autoeval` scripts, `new.sdf`, `newname.sdf` and `test.world`.

[1.0.0]: https://github.com/AdilShaikh1/Robot_manipulator/releases/tag/v1.0.0
