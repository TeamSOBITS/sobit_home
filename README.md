<a name="readme-top"></a>

[JA](README_ja.md) | [EN](README.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# SOBIT HOME


<!-- INTRODUCTION -->
## Introduction

![SOBIT HOME](sobit_home/docs/img/sobit_home.png)

This package is for operating the SOBITS custom mobile manipulator, which combines a four-wheel independent steering drive mechanism, a lift mechanism, dual arms, and a pan-tilt mechanism.

> [!CAUTION]
> If you have no previous experience controlling this robot, please have a senior colleague accompany you while you want to use this robot.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- GETTING STARTED -->
## Getting Started

This section describes how to set up this repository.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Prerequisites

First, please set up the following environment before proceeding to the next installation stage.

| System  | Version |
| --- | --- |
| Ubuntu | 24.04 (Noble Numbat) |
| ROS    | Jazzy Jalisco |
| Python | 3.12 |
| Docker | latest |

> [!NOTE]
> If you need to install `Ubuntu` or `ROS`, please check our [SOBITS Manual](https://github.com/TeamSOBITS/sobits_manual#%E9%96%8B%E7%99%BA%E7%92%B0%E5%A2%83%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6).

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Installation

1. Go to the `src` folder of ROS.
    ```sh
    $ cd ~/colcon_ws/src/
    ```

2. Clone this repository.
    ```sh
    $ git clone https://github.com/TeamSOBITS/sobit_home
    ```

3. Navigate into the repository.
    ```sh
    $ cd sobit_home/
    ```

4. Install the dependent packages.
    ```sh
    $ bash install.sh
    ```

5. Setup Rust for `rm_motors_ros` before compiliing.
    ```sh
    source $HOME/.bashrc

    cd ~/colcon_ws/src/rm_motors_ros/rm_motors_hw/rm_motors_can
    cargo install cargo-expand
    cargo build --release
    ```

6. Compile the package.
    ```sh
    $ cd ~/colcon_ws/
    $ colcon build --symlink-install
    $ source ~/colcon_ws/install/setup.sh
    ```

7. One-time, on the robot's NUC: apply the real-time host tuning (IRQ affinity, CPU
   power profile, DDS UDP buffers). See [REALTIME_SETUP.md](REALTIME_SETUP.md) for
   what it does.
    ```sh
    $ cd ~/colcon_ws/src/sobit_home/realtime_setup
    $ sudo bash setup.sh
    ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- LAUNCH AND USAGE EXAMPLES -->
## Launch and Usage

1. Select features with launch arguments and execute [real_minimal.launch.py](sobit_home_bringup/launch/real_minimal.launch.py) in your **development environment**.
    ```sh
    $ ros2 launch sobit_home_bringup real_minimal.launch.py
    ```

2. You can enable/disable modules directly from CLI (recommended).
    ```sh
    $ ros2 launch sobit_home_bringup real_minimal.launch.py \
      enable_mobile_base:=true \
      enable_body:=true \
      enable_arm_left:=true \
      enable_arm_right:=false \
      enable_head:=true \
      enable_lidar:=true \
      use_rviz:=true
    ```

3. For real hardware mode, load `.bashrc` and set SOBIT HOME domain before launch.
    ```sh
    $ source ~/.bashrc
    $ sobit_home_mode
    ```

If you did not succeed in connecting to the real robot, check the following points:
- Ensure the emergency stop button is not pressed.
- Verify the battery is sufficiently charged.
- Confirm the USB hub is connected to the computer.
- Verify that required environment variables are set in your shell (`DXL_X_LOWER_PORT`, `DXL_X_UPPER_PORT`, `DXL_P_UPPER_PORT`, `UM_PORT`, `HOME_CAM_LEFT_PORT`, `HOME_CAM_RIGHT_PORT`).
- Verify CAN is available (`can0`) when `enable_mobile_base:=true`.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- CONTROL NOTES -->
## Control Notes

### Wheel Controller

`move_wheel_linear` drives the robot straight by a target distance (metres). `move_wheel_rotate` turns in place by a target angle (radians). Both actions use closed-loop PID feedback from odometry and clamp velocity near the goal for smooth stopping. Gains can be tuned at runtime via ROS parameters — no recompile needed.

### Swerve Drive

The four-wheel independent steering controller computes per-wheel steer angle and drive velocity from any combination of `cmd_vel` components (x, y, θ). It automatically picks the shortest steering path for each wheel, reversing the drive direction when that is faster than a full rotation.

### MoveIt Integration

`plan_to_pose` plans a trajectory for the given planning group (`arm_left`, `arm_right`, `arm_left_body`, `arm_right_body`) and caches it. `execute_plan` replays the cached trajectory. For whole-body groups (`arm_left_body`, `arm_right_body`), the base is moved in parallel with the arm via the `MoveItWholeBodyBridge`, which tracks the planned base waypoints using odometry feedback.

`plan_to_named_pose` does the same but takes the name of a state declared in the SRDF (for example `initial_pose` or `move_pose`) instead of a target pose. The resulting plan is cached and executed with `execute_plan` in exactly the same way.

The groups the server initializes come from the `active_planning_groups` parameter. Set it at launch to plan for other groups declared in the SRDF, such as `arm`, `head_arm_body` or the `mobile_base_*` whole-body groups.

All MoveIt interfaces work correctly in both real-hardware and Gazebo simulation modes, including when the simulator is paused.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Runtime-tunable Parameters

The parameters below are re-read while the node runs, so they can be tuned with `ros2 param set` without a relaunch or a rebuild.

`wheel_action_server` — closed-loop wheel motion:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `wheel_linear_kp` / `_ki` / `_kd` | set at launch | PID gains for `move_wheel_linear` |
| `wheel_rotate_kp` / `_ki` / `_kd` | set at launch | PID gains for `move_wheel_rotate` |
| `wheel_linear_arrival_tol` | `0.02` | Arrival tolerance [m] |
| `wheel_rotate_arrival_tol` | `0.02` | Arrival tolerance [rad] |
| `wheel_max_linear_vel` | `0.2` | Velocity clamp [m/s] |
| `wheel_max_lateral_vel` | `0.2` | Lateral velocity clamp [m/s] |

`moveit_server` — planning budget and workspace, defaults in [moveit_server.yaml](sobit_home_moveit_config/config/moveit_server.yaml):

| Parameter | Default | Meaning |
| --- | --- | --- |
| `plan_time_sec` | `10.0` | Wall-clock budget per planning attempt [s] |
| `plan_attempts` | `10` | Number of OMPL solve attempts |
| `workspace_min_x/y/z` | `-5.0`, `-5.0`, `0.0` | Planning workspace lower corner [m] |
| `workspace_max_x/y/z` | `5.0`, `5.0`, `5.0` | Planning workspace upper corner [m] |

```sh
# Tighten the arrival tolerance and shorten the planning budget, live
$ ros2 param set /sobit_home/wheel_action_server wheel_linear_arrival_tol 0.01
$ ros2 param set /sobit_home/moveit_server plan_time_sec 5.0
```

Pass `moveit_server_config:=<path>` at launch to load a different planning-parameter YAML instead of the package default.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Planning Scene Warehouse

MoveIt can persist planning scenes, robot states and constraints in a database, which is what RViz's **Stored Scenes** and **Stored States** panels read and write. `move_group` connects to it directly, so the warehouse is available as soon as the robot is launched.

| Backend | `warehouse_backend` | Storage | Notes |
| --- | --- | --- | --- |
| SQLite | `sqlite` (default) | Single file, `warehouse_database_path` | Installed by `install.sh`; no server process |
| MongoDB | `mongo` | Data directory, served by `mongod` | Built from source, see below |

```sh
# Default: SQLite at ~/.ros/sobit_home_warehouse.sqlite
$ ros2 launch sobit_home_bringup gz_minimal.launch.py

# Choose the file explicitly
$ ros2 launch sobit_home_bringup gz_minimal.launch.py \
  warehouse_database_path:=$HOME/.ros/my_scenes.sqlite
```

To populate an empty database with the default contents, run the warehouse launch once:

```sh
$ ros2 launch sobit_home_moveit_config warehouse_db.launch.py
```

MongoDB has no binary release for ROS 2 Jazzy and no rosdep rule, so it is opt-in and built from source:

```sh
$ WAREHOUSE_MONGO=1 ./install.sh
$ colcon build --packages-select warehouse_ros_mongo
# Start the database server, then launch with the matching backend
$ ros2 launch sobit_home_moveit_config warehouse_db.launch.py warehouse_backend:=mongo
$ ros2 launch sobit_home_bringup gz_minimal.launch.py warehouse_backend:=mongo
```

> [!WARNING]
> With the SQLite backend, saving a scene under a name that already exists throws `warehouse_ros_sqlite::InternalError` (`no such column: M_planning_scene_id`) and aborts the calling process. This is an upstream defect in `warehouse_ros_sqlite` 1.0.5: metadata columns are added lazily, so the tables that never received a `planning_scene_id` do not have that column when the overwrite path queries it. Saving under a **new** name and loading work normally. `move_group` itself is unaffected.

> [!NOTE]
> `warehouse_backend` must be the same for `warehouse_db.launch.py` and the robot launch, otherwise each opens a different store.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Semantic Description (SRDF)

The semantic description is generated from a single [sobit_home.srdf.xacro](sobit_home_moveit_config/config/sobit_home.srdf.xacro) instead of one static SRDF per configuration. It takes the same `enable_*` module flags as the launch files, so a robot brought up without a limb does not advertise planning groups, group states, end effectors or collision pairs that reference links it does not have.

`enable_teleop:=true` drops the `mobile_base` planning groups and the planar virtual joint, because in teleoperation the operator commands the base directly and it must not belong to a planning group.

```sh
$ ros2 launch sobit_home_bringup gz_minimal.launch.py enable_teleop:=true
```

> [!IMPORTANT]
> The URDF and the SRDF are expanded with the same flags, so both describe the same robot. Passing module flags only to one of them makes MoveIt plan against a robot that was never spawned.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Visualize on Rviz2

As a preliminary step to running the actual machine, SOBIT HOME can be visualized on Rviz to display the robot's configuration.

```sh
$ ros2 launch sobit_home_description display.launch.py
```

<!-- If it works correctly, Rviz will be displayed as follows.
![SOBIT HOME Display with Rviz](sobit_home/docs/img/sobit_home_rviz.png) -->

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Run on Gazebo Sim

SOBIT HOME has a simulation environment with Gazebo Harmonic, allowing you to verify operations even without the actual machine.

```sh
$ ros2 launch sobit_home_bringup gz_minimal.launch.py
```

At present, the following virtual environments are available.

| World Name   | Description |
| ------------ | ----------- |
| `empty`        | Spawns an environment without furniture or obstacles. |
| `wrs`          | Spawns the Tidy Up environment used in WRS2020. |
| `small_house`  | Spawns a small house layout developed by AWS. |
| `rcjo2025_arena` | Spawns the RCJ Open 2025 arena world. |
| `rcjo2026_arena` | Spawns the RCJ Open 2026 arena world (default). |

To change the environment, modify the `world_model` parameter in [gz_minimal.launch.py](sobit_home_bringup/launch/gz_minimal.launch.py).

```sh
$ ros2 launch sobit_home_bringup gz_minimal.launch.py world_model:=empty
```

<!-- If it works correctly, the following Gazebo screen will be displayed.
![SOBIT HOME Gazebo Harmonic](sobit_home/docs/img/sobit_home_gz_sim.png) -->

> [!TIP]
> Since it is equipped with sensors similar to the actual machine, the processing may become heavy depending on the computer. Please select only the necessary sensors in [gz_minimal.launch.py](sobit_home_bringup/launch/gz_minimal.launch.py).

```python
'enable_head_cam_color'       : 'true',
'enable_head_cam_depth'       : 'true',
'enable_hand_left_cam_color'  : 'true',
'enable_hand_right_cam_color' : 'true',
'enable_lidar'                : 'true',
```

Additionally, multiple SOBIT HOMEs can be spawned in the same simulation environment by launching additional instances with different `robot_id` and spawn coordinates.

```sh
# Robot 1
$ ros2 launch sobit_home_bringup gz_minimal.launch.py \
  robot_name:=sobit_home robot_id:=1 robot_coords_x:=0.0 robot_coords_y:=0.0 robot_coords_Y:=0.0

# Robot 2
$ ros2 launch sobit_home_bringup gz_minimal.launch.py \
  robot_name:=sobit_home robot_id:=2 robot_coords_x:=0.0 robot_coords_y:=2.0 robot_coords_Y:=0.0
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## Software

### Package Overview

| Package | Role | Main Entry Points |
| --- | --- | --- |
| `sobit_home_bringup` | Integrated startup for real robot and Gazebo | `launch/real_minimal.launch.py`, `launch/gz_minimal.launch.py`, `launch/robot.launch.py` |
| `sobit_home_control` | Swerve base control and MoveIt whole-body bridge | `swerve_controller_node`, `moveit_whole_body_bridge_node` |
| `sobit_home_library` | High-level action/service servers (joint, wheel, MoveIt) | `launch/action_server.launch.py`, `joint_action_server`, `wheel_action_server`, `moveit_action_server` |
| `sobit_home_description` | URDF/Xacro model, RViz config, and base world file | `launch/display.launch.py`, `robots/sobit_home_robot.urdf.xacro` |
| `sobit_home_moveit_config` | MoveIt planning configuration and launch | `launch/move_group.launch.py` |
| `sobit_home_kinematics_plugin` | MoveIt kinematics plugin for SOBIT HOME | `sobit_home_kinematics_plugin_description.xml` |

<details>
<summary>Summary of information on SOBIT HOME and related software</summary>


### Joint Controller

This is a summary of information for moving the joints (pan-tilt mechanism, linear mechanism and manipulator) of SOBIT HOME.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


#### Movement Methods

Implemented interfaces in `sobit_home_library`:

1. Actions
   - `move_joint`
   - `move_to_pose`
   - `move_right_hand_to_pose`
   - `move_left_hand_to_pose`

2. Services
   - `get_hand_to_coord/left` — Analytical IK for the left arm. Accepts a target pose in any TF frame and returns joint angles, a success flag, and reachability hints.
   - `get_hand_to_coord/right` — Analytical IK for the right arm. Same interface as left.
   - `get_hand_to_tf/left`
   - `get_hand_to_tf/right`
   - `get_head_to_coord`
   - `get_head_to_tf`
   - `get_finger_angle`

   **Reachability hints (`move_pose`):** When the target is outside the arm workspace, the service returns `success=false` but still populates `move_pose` with the minimum robot adjustments needed to bring the target into reach:

   | Field | Meaning |
   | --- | --- |
   | `position.x` | Base forward/backward shift (m); positive = drive forward |
   | `position.y` | Reserved for future lateral base movement (always 0.0) |
   | `position.z` | Body lift delta (m); positive = lift up, negative = lift down |
   | `orientation` | Yaw to face the target |

3. MoveIt interfaces (launched from `action_server.launch.py`)
   - Service: `plan_to_pose` — plan to a target pose for a planning group
   - Service: `plan_to_named_pose` — plan to a state named in the SRDF (e.g. `initial_pose`, `move_pose`)
   - Action: `execute_plan` — execute the plan cached by either service

4. Published topics
   - `hand_left/grasp_state`, `hand_right/grasp_state` (`std_msgs/Bool`) — grasp detection, published once after each hand motion completes. `true` when at least two fingers stopped short of their commanded angle, which means an object is blocking them; `false` when the fingers reached their target (nothing grasped).

<p align="right">(<a href="#readme-top">back to top</a>)</p>


#### Joints name

The joint names of SOBIT HOME and their constants are listed below.

| Joint Number | Joint Name | Joint Constant Name |
| :---: | --- | --- |
|  0 | head_pan_joint                | - |
|  1 | head_tilt_joint               | - |
|  2 | arm_left_shoulder_tilt_joint  | - |
|  3 | arm_left_upper_roll_joint     | - |
|  4 | arm_left_upper_flex_joint     | - |
|  5 | arm_left_elbow_joint          | - |
|  6 | arm_left_lower_flex_joint     | - |
|  7 | arm_left_wrist_tilt_joint     | - |
|  8 | arm_left_wrist_roll_joint     | - |
|  9 | arm_right_shoulder_tilt_joint | - |
| 10 | arm_right_upper_roll_joint    | - |
| 11 | arm_right_upper_flex_joint    | - |
| 12 | arm_right_elbow_joint         | - |
| 13 | arm_right_lower_flex_joint    | - |
| 14 | arm_right_wrist_tilt_joint    | - |
| 15 | arm_right_wrist_roll_joint    | - |
| 16 | hand_left_finger_l_mcp_joint  | - |
| 17 | hand_left_finger_l_pip_joint  | - |
| 18 | hand_left_finger_l_dip_joint  | - |
| 19 | hand_left_finger_c_mcp_joint  | - |
| 20 | hand_left_finger_c_ip_joint   | - |
| 21 | hand_left_finger_r_pip_joint  | - |
| 22 | hand_left_finger_r_dip_joint  | - |
| 23 | hand_right_finger_l_mcp_joint | - |
| 24 | hand_right_finger_l_pip_joint | - |
| 25 | hand_right_finger_l_dip_joint | - |
| 26 | hand_right_finger_c_mcp_joint | - |
| 27 | hand_right_finger_c_ip_joint  | - |
| 28 | hand_right_finger_r_pip_joint | - |
| 29 | hand_right_finger_r_dip_joint | - |
| 30 | body_lift_joint               | - |
| 31 | wheel_steer_f_l_joint         | - |
| 32 | wheel_steer_f_r_joint         | - |
| 33 | wheel_steer_b_l_joint         | - |
| 34 | wheel_steer_b_r_joint         | - |
| 35 | wheel_drive_f_l_joint         | - |
| 36 | wheel_drive_f_r_joint         | - |
| 37 | wheel_drive_b_l_joint         | - |
| 38 | wheel_drive_b_r_joint         | - |

<p align="right">(<a href="#readme-top">back to top</a>)</p>


#### How to set new poses

Poses can be added and edited in the file [pose_list.yaml](sobit_home_library/config/pose_list.yaml). The format is as follows:

```yaml
/**:
  ros__parameters:
    poses:
      - initial_pose
      - detecting_pose

    initial_pose:
      body_lift               : 0.5
      head_pan                : 0.0
      head_tilt               : 0.0
      arm_left_shoulder_tilt  : 0.0
      arm_left_upper_roll     : 0.0
      arm_left_upper_flex     : 0.0
      arm_left_elbow          : 0.0
      arm_left_lower_flex     : 0.0
      arm_left_wrist_tilt     : 0.0
      arm_left_wrist_roll     : 0.0
      arm_right_shoulder_tilt : 0.0
      arm_right_upper_roll    : 0.0
      arm_right_upper_flex    : 0.0
      arm_right_elbow         : 0.0
      arm_right_lower_flex    : 0.0
      arm_right_wrist_tilt    : 0.0
      arm_right_wrist_roll    : 0.0
...
```  

Add the desired pose name to `poses`, and then set the angles for each joint under the pose name.

> [!NOTE]
> A named-pose move (`move_to_pose`) always commands **all** arm, body, and head joints — not only the ones you changed. Any joint you omit from a pose block defaults to `0.0` and is actively driven there, so always specify the full joint set for each pose. (Use the `move_joint` action if you want to command a single joint and leave the rest holding position.)

<p align="right">(<a href="#readme-top">back to top</a>)</p>


#### Overriding the pose list at launch

`action_server.launch.py` exposes the pose YAML paths as launch arguments, so another package or machine can supply its own poses **without editing `sobit_home_library`**. The defaults point at the library's own `config/`.

| Launch argument | Default |
| --- | --- |
| `pose_config` | `sobit_home_library/config/pose_list.yaml` |
| `right_hand_pose_config` | `sobit_home_library/config/right_hand_pose_list.yaml` |
| `left_hand_pose_config` | `sobit_home_library/config/left_hand_pose_list.yaml` |

```sh
$ ros2 launch sobit_home_library action_server.launch.py \
    pose_config:=/path/to/my_pose_list.yaml
```

`robot.launch.py` forwards the same arguments, so the override can also be applied from the full bringup. The robot bringup can additionally skip starting the action server entirely (so it can be run on another machine) via `enable_action_server:=false`.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


#### Updating poses at runtime (no restart)

Poses are held as ROS parameters, so they can be set live and reloaded without restarting the node. After changing parameters, call the `reload_poses` service to rebuild the in-memory pose set.

```sh
# Set a single value, or load a whole YAML at once
$ ros2 param set /sobit_home/joint_action_server initial_pose.body_lift 0.42
$ ros2 param load /sobit_home/joint_action_server /path/to/new_pose_list.yaml

# Apply the changes
$ ros2 service call /sobit_home/reload_poses std_srvs/srv/Trigger {}
```

The service reply lists the names of all whole-body poses now loaded — use it to confirm your edit registered.

> [!IMPORTANT]
> `reload_poses` rebuilds the active pose set **entirely from the `poses` array**. Updating an existing pose's values takes effect immediately, but to **add a new pose** you must add its name to the `poses` array as well (e.g. `ros2 param set /sobit_home/joint_action_server poses "[initial_pose, ..., my_new_pose]"`) — otherwise its values are ignored and `move_to_pose` will abort with "pose not found". A pose name dropped from the array is removed from the active set on the next reload.
>
> Runtime changes are **not** written back to the YAML file. Once a value is tuned, copy it into `pose_list.yaml` so it persists across restarts.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Wheel Controller

This is a summary of information for moving the SOBIT HOME moving mechanism.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


#### Moving Methods

Implemented action interfaces in `sobit_home_library`:

1. `move_wheel_linear`
2. `move_wheel_rotate`

The wheel action server publishes velocity commands to `cmd_vel` and uses `odom` for feedback.

</details>

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## Hardware

SOBIT HOME is available as open hardware at [OnShape](https://cad.onshape.com/documents/e17931db96792e39eba48d39/w/a81eeb68b7f4ed981ce8878a/e/42d5107e3af255ccdf5ca7e7?renderMode=0&uiState=69ee43ae00a7b5401b55d390).

![SOBIT HOME in OnShape](sobit_home/docs/img/sobit_home_onshape.png)

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<details>
<summary>For more information on hardware, please click here.</summary>

### How to download 3D parts

1. Access Onshape.

> [!NOTE]
> You do not need to create an `OnShape` account to download files. However, if you wish to copy the entire document, we recommend that you create an account.

2. Select the part in `Instances` by right-clicking on it.
3. A list will be displayed, press the `Export` button.
4. In the window that appears, there is a `Format` item. Select `STEP`.
5. Finally, press the blue `Export` button to start the download.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Electronic Circuit Diagram

![SOBIT HOME Circuit](sobit_home/docs/img/sobit_home_circuit.png)

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- ### Robot Assembly

TBD

<p align="right">(<a href="#readme-top">back to top</a>)</p> -->


### Features

TBD

<!-- | Item | Details |
| --- | --- |
| Maximum linear velocity | 0.8[m/s] |
| Maximum Rotational Speed | 0.229[rad/s] |
| Base Maximum Payload | 20[kg] |
| Manipulator Maximum Payload | 1.0[kg] |
| Size (LxWxH) | 400 x 450 x 1000[mm] |
| Weight | 16.0[kg] |
| Remote Controller | PS4 |
| LiDAR | unk |
| RGB-D | RealSense D415 (head), RealSense D405 (hand) |
| Speaker | Jabra Speak 710 |
| Microphone | MKE 400 |
| Actuator (Arm) | 4 x XM540-W150, 6 x XM430-W320 |
| Power Supply | Makita 6.0Ah 18V |
| PC Connection | USB + Wireless (Kachaka) | -->

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Bill of Materials (BOM)

<!-- Actuators, sensors, and compute. Quantities verified against the powered robot (Dynamixel bus scan + live sensor queries); the right hand mirrors the left. Unit prices and links checked 2026-08-30 — † = listed but out of stock / long lead time, ‡ = price unverified (from search listing, page blocked), n/a = no verified seller found in that region. All prices are per unit — multiply by Qty for totals. "login" = MISUMI shows the price only after account login; the link opens the exact configured part code. -->

**Approx. build cost (Japan column, tax incl., per-unit x Qty): required ≈ ¥3,502,141 (electronics ≈ ¥3,130,635 + mechanical ≈ ¥371,506); optional extras ≈ ¥110,299.**
<!-- LiDAR uses the approx quote figure; ‡/† prices included as listed. -->

#### Actuators & Motion (JP subtotal ≈ ¥2,072,415)

| Group | Part | Model Number | Qty | Japan | USA | EU |
| --- | --- | --- | --- | --- | --- | --- |
| Head + Arms | Dynamixel Actuator | XM430-W350-R | 6 | [¥31,504†](https://e-shop.robotis.co.jp/product.php?id=44) | [$333.39](https://www.robotis.us/dynamixel-xm430-w350-r/) | [€325.56](https://www.generationrobots.com/en/402477-xm430-w350-r-dynamixel-servomotor.html) |
| Arms + Mobile base | Dynamixel Actuator | XM540-W270-R | 16 | [¥51,865†](https://e-shop.robotis.co.jp/product.php?id=43) | [$494.39†](https://www.robotis.us/dynamixel-xm540-w270-r/) | [€489.96†](https://www.generationrobots.com/en/402931-dynamixel-xm540-w270-r-servo.html) |
| Mobile base | Dynamixel Actuator | XM540-W150-R | 2 | [¥51,865](https://e-shop.robotis.co.jp/product.php?id=42) | [$494.39](https://www.robotis.us/dynamixel-xm540-w150-r/) | [€489.96](https://www.generationrobots.com/en/402832-servomoteur-dynamixel-xm540-w150-r.html) |
| Hands | Dynamixel Actuator (Hand) | XL330-M288-T | 14 | [¥4,070†](https://e-shop.robotis.co.jp/product.php?id=417) | [$27.49](https://www.robotis.us/dynamixel-xl330-m288-t/) | [€41.95†](https://www.mybotshop.de/DYNAMIXEL-XL330-M288-T_1) |
| Arms | Dynamixel PRO Actuator (Shoulder) | PH54-200-S500-R | 2 | [¥354,200†](https://e-shop.robotis.co.jp/product.php?id=285) | [$3,541.89†](https://www.robotis.us/dynamixel-ph54-200-s500-r/) | [€3,782.06†](https://www.generationrobots.com/en/403321-dynamixel-pro-plus-h54p-200-s500-r-servo.html) |
| Torso | Dynamixel USB Interface | U2D2 | 4 | [¥6,963](https://e-shop.robotis.co.jp/product.php?id=190) | [$36.92](https://www.robotis.us/u2d2/) | n/a |
| Torso | Dynamixel Power Hub | U2D2 PHB Set | 2 | [¥4,180](https://e-shop.robotis.co.jp/product.php?id=325) | [$21.85](https://www.robotis.us/u2d2-power-hub-board-set/) | n/a |
| Mobile base | Wheel Drive Motor | RoboMaster M3508 P19 | 4 | [¥11,500†](https://store.dji.com/jp/product/rm-m3508-p19-brushless-dc-gear-motor) | [$115.00†](https://www.seeedstudio.com/RoboMaster-M3508-P19-Brushless-DC-Gear-Motor-p-2904.html) | n/a |
| Mobile base | Wheel Motor ESC | RoboMaster C620 | 4 | [¥8,900†](https://store.dji.com/jp/product/rm-c620-brushless-dc-motor-speed-controller) | [$89.00†](https://www.seeedstudio.com/RoboMaster-C620-Brushless-DC-Motor-Speed-Controller-p-2905.html) | n/a |
| Torso (lift) | Lift Motor (NEMA 34 CAN servo stepper, brake) | UIrobot UIM8696CAB | 1 | [¥43,706](https://www.amazon.com/dp/B0DK4W9FTY) ([quote](https://jpacontrol.com/products/show-50.html)) | [‡](https://www.amazon.com/UIROBOT-Stepper-Integrated-Controller-24-48VDC/dp/B0FJQPX36P) (UIM8696CA, no brake) | n/a |
| Torso (lift) | Lift Motor Gateway (RS232 to CAN) | UIrobot UIM2513 | 1 | [¥12,806](https://www.amazon.com/UIROBOT-Adapters-Converter-Hardware-Isolation/dp/B0CNSMJHB1) ([quote](https://jpacontrol.com/products/show-29.html)) | [$125.99‡](https://www.amazon.com/UIROBOT-Adapters-Converter-Hardware-Isolation/dp/B0CNSMJHB1) | n/a |
| Torso (lift) | Lift Motor USB-RS232 Adapter (isolated) | UIrobot UIC321H | 1 | [¥4,481](https://us.amazon.com/UIROBOT-Universal-Converter-Industrial-Ultra-Flexible/dp/B0FKY29JP3) ([quote](https://jpacontrol.com/products/show-63.html)) | [‡](https://us.amazon.com/UIROBOT-Universal-Converter-Industrial-Ultra-Flexible/dp/B0FKY29JP3) | n/a |
| Mobile base | USB-CAN Adapter (isolated, CAN FD) | DSD TECH SH-C31G (CANable 2.0 Pro) | 1 | [¥3,849](https://www.amazon.co.jp/dp/B0FHHCSZY8) | n/a | n/a |
| Mobile base | CAN Bus Hub (CANバスハブ) | — | 1 | [¥1,787](https://www.amazon.co.jp/dp/B0G814QVFB) | n/a | n/a |

#### Sensors & AV (JP subtotal ≈ ¥520,883)

| Group | Part | Model Number | Qty | Japan | USA | EU |
| --- | --- | --- | --- | --- | --- | --- |
| Head | RGB-D Camera (Head) | Orbbec Gemini 336L | 1 | [¥68,420](https://www.digikey.jp/ja/product-highlight/o/orbbec/gemini-330-series-stereo-depth-cameras) | [$379.00](https://store.orbbec.com/products/gemini-336l) | [€586.31](https://www.mybotshop.de/Orbbec-Gemini-336L_1) |
| Mobile base | 2D LiDAR | Hokuyo UST-10LX | 2 | [quote (approx ¥165,000)](https://www.hokuyo-aut.co.jp/search/single.php?serial=16) | [$1,200.00](https://acroname.com/store/scanning-laser-rangefinder-ethernet-r359-ust-10lx) | [€1,878.00†](https://www.generationrobots.com/en/401755-hokuyo-ust-10lx-scanning-laser-range-finder.html) |
| Hands | Wrist Camera (AR0234 global shutter, 1920x1200@90fps, 120deg) | ELP-USBGS1200P01-H120 | 2 | [¥6,066](https://www.amazon.co.jp/dp/B08FD2N9WG) | [$70.64‡](https://www.elpcctv.com/elp-2mp-global-shutter-1200p-1080p-90fps-no-distortion-120-degree-usb-camera-p-557.html) | n/a |
| Torso | Speaker (USB/BT speakerphone) | Jabra Speak 710 | 1 | [¥58,800](https://www.amazon.co.jp/dp/B06XX2N987) | [‡](https://www.jabra.com/business/speakerphones/jabra-speak-series/jabra-speak-710) | n/a |
| Head | Microphone | RØDE VideoMic GO II HELIX (VMGOIIH) | 1 | [¥15,455](https://www.amazon.co.jp/dp/B0D6X93C58/) | [$94.99‡](https://www.sweetwater.com/store/detail/VideoMicGo2H--rode-videomic-go-ii-camera-mounted-shotgun-microphone) | [€85.00](https://www.thomann.de/de/rode_videomic_go_ii.htm) |
| Head | Head Display (8in 1280x800 IPS touch, HDMI) | Waveshare 8DP-CAPLCD | 1 | [¥13,097](https://jp.robotshop.com/products/waveshare-8inch-capacitive-touch-display-toughened-glass-1280x800-ips-hdmi) | [$69.99](https://www.waveshare.com/8dp-caplcd.htm) | [€79.90](https://www.botnroll.com/en/5-89/5242-8-0inch-capacitive-touch-display-toughened-glass-1280-800-hdmi-ips-optical-bonding-screen-waveshare-23741.html) |
| Torso (rear display) | Rear Monitor (16in 4K portable, USB-C/HDMI) | Acouto Zen16 Ultra | 1 | [¥22,979](https://www.amazon.co.jp/dp/B0G7H44RGD) | n/a | n/a |

#### Compute & Network (JP subtotal ≈ ¥314,848)

| Group | Part | Model Number | Qty | Japan | USA | EU |
| --- | --- | --- | --- | --- | --- | --- |
| Torso | Ethernet Switch (5-port GbE) | TP-Link TL-SG605 | 1 | [¥2,033](https://www.amazon.co.jp/dp/B0DP2KSNCK) | n/a | n/a |
| Torso | USB Hub (4-port USB3.0, powered) | UGREEN | 2 | [¥2,999](https://www.amazon.co.jp/dp/B08Y8CJKJC) | n/a | n/a |
| Torso | USB Hub (USB-C 6-in-1, 100W PD) | UGREEN Revodok | 2 | [¥2,999](https://www.amazon.co.jp/dp/B0D1XLNWP2) | n/a | n/a |
| Mobile base (lidar) | LAN Extension Connector (RJ45, 2-pack) | UGREEN | 2 | [¥1,274](https://www.amazon.co.jp/dp/B0DMF8G398) | n/a | n/a |
| Torso | PC | Intel NUC 12 Pro Kit NUC12WSHi5 (RNUC12WSHI50000, i5-1240P) | 1 | [¥188,800](https://www.amazon.co.jp/dp/B0BCWDST4J/) | [$519.00](https://www.newegg.com/asus-barebone-systems-mini-pc-12th-gen-intel-core-i5-1240p/p/2SW-000N-00046) | [€444.00†](https://www.alternate.de/ASUS/NUC-13-Pro-Tall-Kit-NUC13ANHi5-Barebone/html/product/100052198) (NUC 13 substitute) |
| Torso | PC RAM (2x16GB DDR4-3200) | Crucial CT2K16G4SFRA32A | 1 | [¥44,491](https://kakaku.com/item/K0001372325/) | [$229.00](https://www.newegg.com/crucial-32gb-ddr4-3200-cas-latency-cl22-laptop-memory/p/N82E16820156263) | [€264.03‡](https://www.amazon.de/dp/B08C4X9VR5) |
| Torso | PC SSD (NVMe 500GB) | Crucial P5 Plus CT500P5PSSD8 (discontinued — successor: T500) | 1 | [¥64,980‡](https://kakaku.com/item/K0001588760/) (T500) | [$72.84†](https://www.sabrepc.com/CT500P5PSSD8-Crucial-S4602797) | [€204.90†](https://www.reichelt.de/de/de/shop/produkt/crucial_t500_pcie_4_0_nvme_m_2_ssd_500_gb-405533) (T500) |

#### Power (JP subtotal ≈ ¥196,376)

| Group | Part | Model Number | Qty | Japan | USA | EU |
| --- | --- | --- | --- | --- | --- | --- |
| Mobile base | Battery 18V 9.0Ah | BL1890 | 5 | [¥26,318](https://makitashop.jp/?pid=189052500) | [$92.99](https://www.vanonbatteries.com/products/for-makita-9000mah-18v-bl1830-bl1840-bl1845-bl1850-bl1860-bl1890-lxt-li-ion-battery2-pack) (Vanon, 2-pack) | [€124.50](https://geizhals.de/makita-bl1890-lxt-werkzeug-akku-18v-1915h4-0-a3589148.html) |
| Mobile base | Battery Connector (18V DIY power connector, 2-pc set) | Gakkiti (Makita compatible) | 2 | [¥1,599](https://www.amazon.co.jp/dp/B094XXC8LL) | n/a | n/a |
| Mobile base | Battery Adapter (2x18V to 36V) | Makita BCV03 (A-57255 / 196809-7) | 1 | [¥14,190](https://makitashop.jp/?pid=108094683) | [$113.99](https://dryitcenter.com/products/makita-36v-adaptor-cordless-bcv03) | [€89.25](https://geizhals.de/makita-bcv03-2x-18v-adapter-fuer-akkus-196809-7-a2202258.html) |
| Offboard (charger) | Battery Charger (2-port rapid; alt: DC18RC / DC18WC) | Makita DC18RD | 1 | [¥16,200](https://search.kakaku.com/dc18rd/) | [$164.99†](https://www.masterwholesale.com/makita-dc18rd-18v-lxt-lithium-ion-dual-port-rapid-optimum-charger.html) | [€71.88](https://geizhals.de/makita-dc18rd-ladegeraet-196933-6-a1292788.html) |
| Mobile base | DC-DC Converter (30-90V to 24V, 20A/480W) | Mzhou buck converter | 1 | [¥7,499](https://www.amazon.co.jp/dp/B0D1G6P599) | n/a | n/a |
| Mobile base | DC-DC Converter (24V to 12V, 30A/360W) | — | 3 | [¥3,799](https://www.amazon.co.jp/dp/B0976VJ5CS) | n/a | n/a |
| Mobile base | DC-DC Converter (12/24V to 5V, 20A/100W, waterproof) | HOMELYLIFE step-down | 2 | [¥4,303](https://www.amazon.co.jp/dp/B089M5R3NJ) | n/a | n/a |
| Mobile base | Screw Terminal Block (8P, 2-row) | — | 1 | [¥3,696](https://www.amazon.co.jp/dp/B0GT8QMDZZ) | n/a | n/a |

#### Cabling & Misc (JP subtotal ≈ ¥26,113)

| Group | Part | Model Number | Qty | Japan | USA | EU |
| --- | --- | --- | --- | --- | --- | --- |
| Torso | HDMI 2.1 Cable (2m, 8K) | UGREEN | 1 | [¥1,601](https://www.amazon.co.jp/dp/B0CFFFSFFN) | n/a | n/a |
| Torso (cabling) | Drag Chain (15x30mm, 1m) | Akozon | 4 | [¥1,492](https://www.amazon.co.jp/dp/B07YG2C1D7) | n/a | n/a |
| Torso (cabling) | USB 3.0 Extension Cable (2m) | UGREEN | 4 | [¥1,038](https://www.amazon.co.jp/dp/B086ZJB2JN) | n/a | n/a |
| Torso (cabling) | USB-C Cable (L-shape, 100W, 4K, 2m) | UGREEN | 2 | [¥2,880](https://www.amazon.co.jp/dp/B08R86PLCS) | n/a | n/a |
| Torso (cabling) | Micro-USB Cable (2m) | UGREEN | 3 | [¥954](https://www.amazon.co.jp/dp/B07VNM61ZL) | n/a | n/a |
| Torso (cabling) | Micro-USB Cable (0.5m) | UGREEN | 1 | [¥674](https://www.amazon.co.jp/dp/B07VQTRYY4) | n/a | n/a |
| Mobile base (lidar, NUC) | LAN Cable (CAT8 mesh, short) | UGREEN | 3 | [¥1,099](https://www.amazon.co.jp/dp/B0CMWFN5R8) | n/a | n/a |
| Mobile base (Remote PC) | LAN Cable (CAT8 mesh, long) | UGREEN | 1 | [¥1,799](https://www.amazon.co.jp/dp/B0CMWFN5R8) | n/a | n/a |

#### Mechanical (JP subtotal ≈ ¥371,506)

| Group | Part | Model Number | Qty | Japan | USA | EU |
| --- | --- | --- | --- | --- | --- | --- |
| Mobile base | Drive Wheel (150mm rubber, 12mm shaft) | Inoac LR-150W-BK-12 | 4 | [¥2,013](https://www.genbaichiba.com/shop/g/g00621419/) | n/a | n/a |
| Mobile base | Timing Pulley (S5M, 20T, 10mm belt) | MISUMI HTPS20S5M100-A-P10 | 8 | [¥3,343](https://jp.misumi-ec.com/vona2/detail/110300406030/?HissuCode=HTPS20S5M100-A-P10) | [$52.67](https://us.misumi-ec.com/vona2/detail/110300406030/?HissuCode=HTPS20S5M100-A-P10) | [€44.84](https://de.misumi-ec.com/vona2/detail/110300406030/?HissuCode=HTPS20S5M100-A-P10) |
| Mobile base | Timing Belt (S5M, 300mm, 10mm wide) | MISUMI HTBN300S5M-100 | 4 | [¥736](https://jp.misumi-ec.com/vona2/detail/110302653030/?HissuCode=HTBN300S5M-100) | [$15.79](https://us.misumi-ec.com/vona2/detail/110302653030/?HissuCode=HTBN300S5M-100) | [€10.36](https://de.misumi-ec.com/vona2/detail/110302566230/?HissuCode=HTBN300S5M-100) |
| Mobile base | Spur Gear (module 1.0, 40T, 8mm face, 10mm bore) | MISUMI GEAB1.0-40-8-B-10 | 4 | [¥1,533](https://jp.misumi-ec.com/vona2/detail/110300428430/?HissuCode=GEAB1.0-40-8-B-10) | [$40.44‡](https://us.misumi-ec.com/vona2/detail/110300428430/?HissuCode=GEAB1.0-40-8-B-10) | n/a |
| Mobile base | Spur Gear (module 1.0, 40T, hub, 15mm bore) | MISUMI GEAHB1.0-40-8-A-15 | 4 | [¥1,477](https://jp.misumi-ec.com/vona2/detail/110300428430/?HissuCode=GEAHB1.0-40-8-A-15) | [‡](https://us.misumi-ec.com/vona2/detail/110300428430/?HissuCode=GEAHB1.0-40-8-A-15) | n/a |
| Mobile base | Miter Gear (module 1.5, 20T 1:1, SUS304) | MISUMI KGTS1.5-2020-10 | 8 | [¥8,973](https://jp.misumi-ec.com/vona2/detail/110300429650/?HissuCode=KGTS1.5-2020-10) | [$130.65‡](https://us.misumi-ec.com/vona2/result/?Keyword=KGTS1.5-2020-10) | n/a |
| Mobile base | Precision Shaft (10mm dia; L=55/60/120/130) | MISUMI PSSFG10-55/-60/-120/-130 | 4 each | [¥485-802](https://jp.misumi-ec.com/vona2/detail/110302634310/?HissuCode=PSSFG10-55) | [$7.96-13.69](https://us.misumi-ec.com/vona2/detail/110302634310/?HissuCode=PSSFG10-55) | [login](https://de.misumi-ec.com/vona2/detail/110302634310/?HissuCode=PSSFG10-55) |
| Mobile base | Bearing Spacer (10x12mm, L=2) | MISUMI CLBUB10-12-2 | 32 | [¥496](https://jp.misumi-ec.com/vona2/detail/110302644450/?HissuCode=CLBUB10-12-2) | [$22.52](https://us.misumi-ec.com/vona2/detail/110302644450/?HissuCode=CLBUB10-12-2) | n/a |
| Mobile base | Bearing Spacer (10x12mm, L=30) | MISUMI CLBUB10-12-30 | 4 | [¥707](https://jp.misumi-ec.com/vona2/detail/110302644450/?HissuCode=CLBUB10-12-30) | [$24.15‡](https://us.misumi-ec.com/vona2/detail/110302644450/?HissuCode=CLBUB10-12-30) | [€7.84](https://de.misumi-ec.com/vona2/detail/110302644450/?HissuCode=CLBUB10-12-30) |
| Mobile base | Shaft Support (flanged, slit, 10mm dia) | MISUMI SSTHMR10 | 12 | [¥2,166](https://jp.misumi-ec.com/vona2/detail/110300013150/?HissuCode=SSTHMR10) | [login](https://us.misumi-ec.com/vona2/detail/110300013150/?HissuCode=SSTHMR10) | [€28.23](https://de.misumi-ec.com/vona2/detail/110300013150/?HissuCode=SSTHMR10) |
| Mobile base | Flanged Bearing (stainless, 10x15x4) | SFL6700ZZ | 48 | [¥1,099](https://jp.misumi-ec.com/vona2/detail/110302273720/?HissuCode=SFL6700ZZ) | [$19.99](https://vxb.com/products/sf6700zz-stainless-steel-flanged-shielded-bearing-10x15x4) | [€5.31](https://www.123kugellager.de/kugellager-gehauselager/rillenkugellager/einreihig/f6700-zz) (steel equiv.) |
| Mobile base | Bearing (40x52x7) | 6808ZZ | 4 | [¥1,804](https://jp.misumi-ec.com/vona2/detail/221000058301/?HissuCode=6808ZZ) | [$19.99](https://vxb.com/products/6808zz-bearing-40x52x7-shielded) | [€3.25](https://www.hug-technik.com/shop/kugellager-61808-2z-von-zen-rillenkugellager-40x52x7-mm.html) (ZEN) |
| Mobile base | Thrust Bearing (55x78x16) | 51111 | 4 | [¥2,400](https://jp.misumi-ec.com/vona2/detail/221000058299/?HissuCode=51111) | [$49.99](https://vxb.com/products/51111-thrust-bearing-55x78x16) | [€7.71](https://www.hug-technik.com/shop/axial-rillenkugellager-51111-von-zen-55x78x16-mm.html) (ZEN) |
| Mobile base | Standoff (SUS303, M4, 45mm, M-F slotted) | Hirosugi BRU-445S | 16 | [¥280](https://www.hirosugi-net.co.jp/shop/g/g41214/) (20-pc min) | n/a | n/a |
| Mobile base | Standoff (SUS303, M4, 50mm, M-F slotted) | Hirosugi BRU-450S | 16 | [¥294](https://www.hirosugi-net.co.jp/shop/g/g41215/) (20-pc min) | n/a | n/a |
| Mobile base | Aluminium Frame (20x20, 472mm) | MISUMI HFS5-2020-472 | 4 | [¥283](https://jp.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-472) | [login](https://us.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-472) | [login](https://de.misumi-ec.com/vona2/result/?Keyword=HFS5-2020) |
| Mobile base | Aluminium Frame (20x20, 432mm) | MISUMI HFS5-2020-432 | 10 | [¥253](https://jp.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-432) | [login](https://us.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-432) | [login](https://de.misumi-ec.com/vona2/result/?Keyword=HFS5-2020) |
| Mobile base | Aluminium Frame (20x20, 236mm) | MISUMI HFS5-2020-236 | 4 | [¥325](https://jp.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-236) | [login](https://us.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-236) | [login](https://de.misumi-ec.com/vona2/result/?Keyword=HFS5-2020) |
| Mobile base | Aluminium Frame (20x20, 78mm) | MISUMI HFS5-2020-78 | 8 | [¥325](https://jp.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-78) | [login](https://us.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-78) | [login](https://de.misumi-ec.com/vona2/result/?Keyword=HFS5-2020) |
| Mobile base (battery tray) | Telescopic Slide Rail (3-step, SUS304, W27) | MISUMI SSRXC2718 | 2 | [¥4,461](https://jp.misumi-ec.com/vona2/detail/110300072560/?HissuCode=SSRXC2718) | [login](https://us.misumi-ec.com/vona2/detail/110300072560/?HissuCode=SSRXC2718) | [login](https://uk.misumi-ec.com/vona2/detail/110300072560/?HissuCode=SSRXC2718) |
| Mobile base (battery tray) | Aluminium Frame (20x20, 200mm) | MISUMI HFS5-2020-200 | 4 | [¥325](https://jp.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-200) | [login](https://us.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-200) | [login](https://de.misumi-ec.com/vona2/result/?Keyword=HFS5-2020) |
| Torso lift | Lift Ball Screw (rolled, 20mm dia, 10mm lead, 860mm) | MISUMI C-BSSTA2010-860 | 1 | [¥24,475](https://jp.misumi-ec.com/vona2/detail/110302588540/?HissuCode=C-BSSTA2010-860) | n/a | n/a |
| Torso lift | Lift Linear Guide Rail (SX, W28, 1000mm) | MISUMI SXR28-1000 | 2 | [¥7,904](https://jp.misumi-ec.com/vona2/detail/110300048850/?HissuCode=SXR28-1000) | [login](https://us.misumi-ec.com/vona2/detail/110300048850/?HissuCode=SXR28-1000) | [login](https://uk.misumi-ec.com/vona2/detail/110300048850/?HissuCode=SXR28-1000) |
| Torso lift | Lift Coupling (disc type, OD32, 12/14mm bores) | MISUMI MCSLCRK32-12-14 | 1 | [¥4,694](https://jp.misumi-ec.com/vona2/detail/110302556340/?HissuCode=MCSLCRK32-12-14) | [login](https://us.misumi-ec.com/vona2/detail/110302556340/?HissuCode=MCSLCRK32-12-14) | [login](https://uk.misumi-ec.com/vona2/detail/110302556340/?HissuCode=MCSLCRK32-12-14) |
| Torso lift | Lift Ball Screw Support Unit | MISUMI C-TFF15 | 1 | [¥2,376](https://jp.misumi-ec.com/vona2/detail/110310392309/?HissuCode=C-TFF15) | n/a | n/a |
| Torso frame | Aluminium Frame (20x20, 600mm) | MISUMI HFS5-2020-600 | 1 | [¥364](https://jp.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-600) | [$9.42‡](https://us.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-600) | [login](https://de.misumi-ec.com/vona2/result/?Keyword=HFS5-2020) |
| Torso frame | Aluminium Frame (20x20, 100mm) | MISUMI HFS5-2020-100 | 6 | [¥325](https://jp.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-100) | [$4.66‡](https://us.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-100) | [login](https://de.misumi-ec.com/vona2/result/?Keyword=HFS5-2020) |
| Torso frame | Aluminium Frame (20x20, 110mm) | MISUMI HFS5-2020-110 | 1 | [¥325](https://jp.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-110) | [$4.66‡](https://us.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-110) | [login](https://de.misumi-ec.com/vona2/result/?Keyword=HFS5-2020) |
| Torso (back) | Emergency Stop Button | IDEC HW1B-V404R | 1 | [¥3,504](https://jp.misumi-ec.com/vona2/detail/222000393180/?HissuCode=HW1B-V404R) | [login](https://us.misumi-ec.com/vona2/detail/222000393180/?HissuCode=HW1B-V404R) | n/a |
| Arms + steer (X540 joints) | Flanged Bearing (12x18x4) | F6701ZZ | 18 | [¥1,239](https://jp.misumi-ec.com/vona2/detail/221000529012/?HissuCode=F6701ZZ) | [$6.19](https://bearingsdirect.com/f6701-zz-flanged-ball-bearing-12x18x4mm-shielded/) | [€2.20‡](https://rcbay.de/Kugellager-mit-Bund-F6701-ZZ-12x18x4-mm-Flanschlager-Bundlager) |
| Arms (shoulder) | Bearing (50x65x7) | 6810ZZ | 6 | [¥2,330](https://jp.misumi-ec.com/vona2/detail/221000058301/?HissuCode=6810ZZ) | [$39.99](https://vxb.com/products/6810-open-bearing-50x65x7) (open) | [€4.52](https://www.hug-technik.com/shop/kugellager-61810-von-zen-rillenkugellager-50x65x7-mm.html) (ZEN, open) |
| Arms (joints) | Bearing (20x27x4) | 6704ZZ | 6 | [¥1,268](https://jp.misumi-ec.com/vona2/detail/221000058301/?HissuCode=6704ZZ) | [$9.99](https://vxb.com/products/6704zz-shielded-bearing-20x27x4) | [€10.59](https://www.123kugellager.de/kugellager-gehauselager/rillenkugellager/einreihig/61704-zz) |
| Arms (upper arm) | Aluminium Frame (20x20, 180mm; verified from CAD) | MISUMI HFS5-2020-180 | 2 | [¥325](https://jp.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-180) | [login](https://us.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-180) | [login](https://de.misumi-ec.com/vona2/result/?Keyword=HFS5-2020) |
| Arms (forearm) | Aluminium Frame (20x20, 330mm; verified from CAD) | MISUMI HFS5-2020-330 | 2 | [¥191](https://jp.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-330) | [login](https://us.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-330) | [login](https://de.misumi-ec.com/vona2/result/?Keyword=HFS5-2020) |

#### Optional (JP subtotal ≈ ¥110,299)

| Group | Part | Model Number | Qty | Japan | USA | EU |
| --- | --- | --- | --- | --- | --- | --- |
| (Optional) Torso | USB-C GbE LAN Adapter | UGREEN | 1 | [¥2,099](https://www.amazon.co.jp/dp/B082K62S48) | n/a | n/a |
| (Optional) Torso | Mobile WiFi Router (+ home kit) | Fujisoft +F FS040W | 1 | [¥33,000](https://www.amazon.co.jp/dp/B09HRH6XBL) | n/a | n/a |
| (Optional) Offboard (teleop) | Game Controller | Sony DualShock 4 (CUH-ZCT2J) | 1 | [¥15,800](https://www.amazon.co.jp/dp/B01LPTFJ8W) | n/a | n/a |
| (Optional) Offboard (teleop) | VR Headset | Meta Quest 3S 128GB | 1 | [¥59,400](https://www.amazon.co.jp/dp/B0F8VJ57Q1) | n/a | n/a |

<!-- Servo placement (from bus scan): head pan/tilt = 2x XM430-W350; per arm = 1x PH54-200 (shoulder) + 7x XM540-W270 + 2x XM430-W350 (wrist); per hand = 7x XL330-M288; wheel steering = 2x XM540-W150 + 2x XM540-W270. NUC12WSHi5, the DDR4 kit, and the RoboMaster parts are end-of-life — remaining stock is priced above historical street price. -->

<!-- Remaining items to be added: USB extension cables, power cables, power hubs, screws/nuts, 3D-print filament. -->

> [!IMPORTANT]
> Prices may vary depending on the retailer. Please check each link for the latest prices.

</details>

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- MILESTONE -->
## Milestone

- [ ] Hardware [Features](#features) — fill in the specification table (velocities, payloads, size, weight, sensors, actuators, power)
- [ ] [Bill of Materials (BOM)](#bill-of-materials-bom) — complete the parts list with model numbers, quantities, costs and purchase links
- [ ] Robot Assembly — assembly procedure

Both sections currently read `TBD` and have a draft table commented out in the Markdown source; the values in those drafts still need to be measured and verified against the current build before they are published.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- ACKNOWLEDGMENTS -->
## References

* [RM Motors HW](https://github.com/mjforan/rm_motors_ros)
* [Dynamixel Hardware](https://github.com/dynamixel-community/dynamixel_hardware)
* [ROS Jazzy](https://docs.ros.org/en/jazzy/index.html)
* [ROS2 Control](https://control.ros.org/jazzy/index.html)
* [ROS2 Control Gazebo](https://github.com/ros-controls/gz_ros2_control)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/sobit_home.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/sobit_home/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/sobit_home.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/sobit_home/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/sobit_home.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/sobit_home/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/sobit_home.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/sobit_home/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/sobit_home.svg?style=for-the-badge
[license-url]: LICENSE
