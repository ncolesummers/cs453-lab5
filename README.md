# CS 453 Lab 5: Docking

This lab serves as an introduction to ROS2 using the Create3 from iRobot. The Create3 is a mobile robot that can be controlled using ROS2. The Create3 has a docking station that it can autonomously dock with. This lab will focus on using the Create3 to dock with the docking station.

Steps:

    1. Undock
    2. Drive 1m
    3. Turn randomly
    4. Drive 0.5m
    5. Dock without the Dock action

Additional Requirements:

- Add chirp tones between transitions. For Example, before you drive, play a tone to signify you’re driving.

- Different tones between steps might be a helpful debugging tool!

- Fully docked is: /dock_status.is_docked = True

## Running the Code

### Requirements

- ROS2 and the Create3 ROS2 packages must be installed. See the [Create3 ROS2 Setup](https://iroboteducation.github.io/create3_docs/setup/ubuntu2204/) for guidance.

- This Repo 

### Build

Use colcon to build the packages

```bash
$ colcon build
```

### Run

1. source the ROS2 environment:

```bash
$ source install/setup.bash
```

2. Spin up the dockStatus node:

```bash
$ ros2 run dockStatus dockStatus
```

3. Spin up the dock node:

```bash
$ ros2 run dock dock
```
## About

### Constants

The dictionary constants at the top of the dock.py file are inspired by [Design Tokens](https://designsystem.digital.gov/design-tokens/). 