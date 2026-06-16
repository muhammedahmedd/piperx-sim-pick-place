# Piper X Pick-and-Place

This repository contains a ROS 2 Humble pick-and-place project for the Piper X arm using MoveIt 2, Isaac Sim, and ArUco marker detection.

In Isaac Sim, the robot detects a cube with an ArUco tag on it, picks it up, and places it on a table at a target location marked by a second ArUco tag. In testing, the average placing error was 6 mm.

The pick-and-place sequence is defined using a finite state machine (FSM), which moves the robot through the main stages of scanning, picking, lifting, placing, and completing the task in smaller steps.

oveIt 2 is used for motion planning and execution, allowing the robot to plan and execute arm motions to the detected cube pose and target placement pose, while Isaac Sim provides the simulated robot, camera, and the scene.

## Clone the repository

```bash
git clone https://github.com/muhammedahmedd/piperx-pick-place.git ~/piperx_ws
cd ~/piperx_ws
```

## External dependencies

This project depends on a Piper X-focused fork of `agx_arm_sim`. This fork contains the Piper X MoveIt configuration used with Isaac Sim.

I modified this fork to synchronize the MoveIt setup with Isaac Sim simulation time. This includes updating MoveIt-related launch files so nodes such as `move_group`, `robot_state_publisher`, and `ros2_control_node` use Isaac Sim clock time from `/clock`.

This fork also points to my Piper X URDF fork, where I added a fixed `gripper_tcp` frame. The `gripper_tcp` frame is used as the grasp reference frame for the gripper during pick-and-place.

Clone it into the workspace `src/` folder:

```bash
cd ~/piperx_ws
git clone --recursive https://github.com/muhammedahmedd/piperx_arm_sim.git src/agx_arm_sim
```

The `--recursive` flag is important because `piperx_arm_sim` contains a URDF submodule. If you cloned it without `--recursive`, initialize the submodule manually:

```bash
cd ~/piperx_ws/src/agx_arm_sim
git submodule update --init --recursive
```

For more details about the MoveIt simulation-time changes and the added `gripper_tcp` frame, see the README files inside the `piperx_arm_sim` and `piperx_arm_urdf` repositories.

## Docker setup

This project includes a Docker setup for running the ROS 2 Humble and MoveIt 2 environment used by the pick-and-place system.

The Docker image installs the ROS 2 and MoveIt dependencies needed for this project, including `topic_based_ros2_control`, which allows the Piper X MoveIt configuration to communicate with Isaac Sim through `/isaac_joint_command` and `/isaac_joint_states`.

Build the Docker image from the workspace root:

```bash
cd ~/piperx_ws
docker compose build
```

Start the container:

```bash
docker compose up -d
```

Enter the running container:

```bash
docker exec -it ros2_humble bash
```

Inside the container, the workspace is available at:

```text
/workspace/piperx_ws
```

This works because `compose.yaml` mounts the workspace from the host machine into the container. If your workspace is not located at `~/piperx_ws`, update the volume path in `compose.yaml`.

## Build the ROS 2 workspace

Inside the container:

```bash
cd /workspace/piperx_ws
colcon build
source install/setup.bash
```

## Run MoveIt

Start the Isaac Sim scene first. After Isaac Sim is running, launch MoveIt and RViz:

```bash
ros2 launch piper_x_gripper_moveit_config demo.launch.py
```

The Piper X MoveIt configuration in `piperx_arm_sim` already includes the Isaac Sim topic-based ROS 2 control setup, so users do not need to manually modify the MoveIt hardware configuration.

## Run the pick-and-place pipeline

After Isaac Sim is running and MoveIt has been launched, start the pick-and-place pipeline:

```bash
ros2 launch piperx_control pick_place_sim.launch.py
```

This launch file starts the ArUco perception node and the pick-and-place control node:

```text
piperx_perception/aruco_sim_detector
piperx_control/piperx_sim_control
```

The `aruco_sim_detector` node detects the cube ArUco marker and the place ArUco marker from the Isaac Sim camera image and publishes their poses relative to the base_link frame. The `piperx_sim_control` node runs the finite state machine (FSM) that moves the robot through the pick-and-place sequence.

### Launch parameters

The launch file includes three configurable parameters:

| Parameter                   | Default | Description                                                                              |
| --------------------------- | ------: | ---------------------------------------------------------------------------------------- |
| `marker_size`               | `0.055` | ArUco marker side length in meters.                                                      |
| `settle_velocity_threshold` | `0.033` | Joint velocity threshold used to decide when the arm has settled after a motion.         |
| `place_tcp_z`               | `0.040` | TCP z height used when placing the object.                                               |

You can override these parameters from the command line:

```bash
ros2 launch piperx_control pick_place_sim.launch.py marker_size:=0.055 settle_velocity_threshold:=0.033 place_tcp_z:=0.10
```

## Notes

The Docker image provides the ROS 2 and MoveIt environment. The project workspace is mounted into the container so code changes can be made on the host machine and built inside the container.
