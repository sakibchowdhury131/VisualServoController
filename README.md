# VisualServoController

A ROS 1 Noetic package implementing a visual servoing controller for the Kinova Jaco2 6-DOF robotic arm. The system uses computer vision feedback to control the real robot arm, enabling it to track and reach targets identified in camera images.

## Features

- Visual servoing control loop for real Kinova Jaco2 arm
- ROS 1 Noetic node architecture (CMake/catkin package)
- Vision-based controller script (`vision-controller.py`)
- MoveIt integration for motion planning
- Designed for deployment on physical hardware

## Tech Stack

- Python 3
- ROS 1 Noetic
- MoveIt
- Kinova Jaco2 ROS driver
- OpenCV (vision processing)
- CMake / catkin

## Project Structure

| File / Directory | Description |
|---|---|
| `scripts/vision-controller.py` | Main visual servoing controller node |
| `scripts/moveit/` | MoveIt motion planning utilities |
| `kinova_arm_control/` | Kinova arm control interface |
| `CMakeLists.txt` | Catkin build configuration |
| `package.xml` | ROS package metadata |

## Requirements

- ROS 1 Noetic
- Kinova Jaco2 ROS driver (`kinova-ros`)
- MoveIt
- Python 3, OpenCV

## Setup

1. Clone into your catkin workspace `src/` directory.
2. Install Kinova ROS driver and MoveIt.
3. Build the workspace:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

4. Connect the Kinova Jaco2 arm and launch the driver.
5. Run the visual servo controller:

```bash
rosrun VisualServoController vision-controller.py
```

## License

MIT
