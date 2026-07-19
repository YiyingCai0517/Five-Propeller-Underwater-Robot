# ROS1 URDF visualization

`simulatedrobot/` is a SolidWorks-exported ROS1 package containing URDF, STL meshes, and launch files.

## Recommended visualization

Copy or symlink `simulatedrobot` into a catkin workspace `src/` directory, build the workspace, source it, and launch the corrected visualization variant:

```bash
catkin_make
source devel/setup.bash
roslaunch simulatedrobot display_fixed.launch
```

For Gazebo geometry spawning:

```bash
roslaunch simulatedrobot gazebo_fixed.launch
```

The `*_fixed` files use `simulatedrobot_fixed.urdf`, which corrects the obviously invalid zero inertia of `rfw` and the likely base inertial coordinate swap while preserving the original export for comparison.

## Known limitations

- The package targets ROS1 and has not been migrated to ROS2.
- The original `display.launch` references a missing `urdf.rviz` configuration.
- Detailed visual meshes are also used for collision geometry.
- Inertial values require physical/CAD verification.
- There are no buoyancy, hydrodynamic drag, added-mass, current, thruster, battery, communication, IMU, or depth-sensor plugins.
- Spawning the model in Gazebo is not equivalent to an underwater dynamics simulation.

Use `simulation/python/` for the standalone numerical task examples, with the limitations documented there.
