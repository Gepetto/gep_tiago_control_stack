This repository illustrates the use of **Crocoddyl** and **Pinocchio** for optimal control on mobile manipulators, demonstrated on a PAL Robotics Tiago robot.
The version considered here is a differential mobile base equipped with a torso and a seven DoFs arm.

## Packages descriptions 
### tiago_lfc_bringup
Provides environment setup to launch Tiago in Gazebo simulation. Also includes useful launch files for controller management.

## tiago_simple_mpc
MPC implementation example using Crocoddyl for Tiago control.

Two nodes are available once simulation is running:
- `test_ocp_reaching.py` - Visualizes OCP solutions offline (uses meshcat)
- `cartesian_target_mpc_node.py` - Runs MPC in closed-loop with Gazebo

## Installation

Create workspace and clone repository

```bash
mkdir -p tiago_ws/src 
cd tiago_ws/src
git clone git@github.com:Gepetto/gep_tiago_control_stack.git
vcs import . < tiago_control_stack/tiago_lfc_bringup/dependencies/lfc_dependencies.repos
vcs import . < tiago_control_stack/tiago_lfc_bringup/dependencies/tiago_dependencies.repos
cd ..
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# install crocoddyl
export MAKEFLAGS="-j2"
colcon build --packages-up-to crocoddyl --event-handlers console_direct+ --cmake-args -DBUILD_PYTHON_BINDINGS=ON

# install linear feedback controller
colcon build --symlink-install \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_DISABLE_FIND_PACKAGE_Doxygen=ON \
    --packages-select tiago_lfc \
                      linear_feedback_controller_msgs \
                      linear_feedback_controller

# install all other packages
colcon build --symlink-install
```

## Usage 

### Start Tiago, simulated in Gazebo Harmonic and add lfc controllers

You can use simple launch file to set up environment. 
Useful launch files can be found under directory tiago_lfc/launch

```bash
# launch tiago in gazebo
ros2 launch tiago_lfc tiago_gazebo.launch.py world_name:=house # or empty
# Load lfc controllers
ros2 launch tiago_lfc switch_to_lfc_controllers.launch.py
```

> [!note]
> 3 controllers are packaged together and are in chained mode.
> hardware -> Joint-state-estimator -> Linear-feedback-controller -> Passthrough-controller -> hardware

### Use high level launch file

Concreate application can be found in launch folder at the root of the package.

|**Launch file**                  | **Description**                                                                                     |
|---------------------------------|---------------------------------------------------------------------------------------------------|
|`pd_plus_controller.launch.py`   |Use pd plus exemple found in lfc package.



## Dependencies

gep_tiago_control_stack is based on the Tiago harmonic project developed by
Juan Carlos Manzanares Serrano, Francisco Martín Rico, and Juan Sebastían Cely Gutiérrez.
We are very thankful to them for providing this open source simulation based on Gazebo Harmonic.

gep_tiago_control_stack is also using heavily ros2_control, and we also thanks the ros2_control community for their effort.

## License

This software is distributed under the terms of both the MIT license and the Apache License (Version 2.0).

See LICENSE-APACHE and LICENSE-MIT for details.

## Contributors
Clément Pène

## Funding Agency
Défi-Clef Robotique Centrée sur l'humain
