# ATF for ipa_navigation_planning repository

Please view the [README](../README.md) in the parent directory for information regarding the ATF.

### Layout

This atf subdirectory implements an ATF test to record bagfiles of the robot following a predefined route (see [robot-environment-config](#####Robot Environment Config)) containing data relating to the current robot position (state_ekf) and the current velocity (odometry). This information can then be presented in a 2D-plot with the atf_bagfile_plotter.

### How-to

Please follow the installation steps described in the installation section of the aforementioned ATF README; then checkout this branch and run
```sh
catkin_make --force-cmake
```

### Config Files
##### Robot Environment Config
##### Robot Config
##### Test Config
##### Test Generation Config
##### Test Suites
##### Bagfile Plotter Config
