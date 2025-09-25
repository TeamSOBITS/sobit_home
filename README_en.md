<a name="readme-top"></a>

[JA](README.md) | [EN](README_en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# SOBIT HOME

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#introduction">Introduction</a>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li>
    　<a href="#launch-and-usage">Launch and Usage</a>
      <ul>
        <li><a href="#visualization-on-rviz<">Visualization on Rviz</a></li>
        <li><a href="#run-on-gazebo-sim<">Run on Gazebo Sim</a></li>
      </ul>
    </li>
    <li>
    　<a href="#software">Software</a>
      <ul>
        <li><a href="#joint-controller">Joint Controller</a></li>
        <li><a href="#wheel-controller">Wheel Controller</a></li>
      </ul>
    </li>
    <li>
    　<a href="#hardware">Hardware</a>
      <ul>
        <li><a href="#how-to-download-3d-parts">How to download 3D Parts</a></li>
        <li><a href="#electronic-circuit-diagram">Electronic circuit Diagram</a></li>
        <!-- <li><a href="#robot-assembly">Robot Assembly</a></li> -->
        <li><a href="#features">Features</a></li>
        <li><a href="#bill-of-material-BOM">Bill of Material (BOM)</a></li>
      </ul>
    </li>
    <li><a href="#milestone">Milestone</a></li>
    <!-- <li><a href="#contributing">Contributing</a></li> -->
    <!-- <li><a href="#license">License</a></li> -->
    <li><a href="#references">References</a></li>
  </ol>
</details>



<!-- INTRODUCTION -->
## Introduction

<!-- ![SOBIT HOME](sobit_home/docs/img/sobit_home.png) -->

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

5. Compile the package.
    ```sh
    $ cd ~/colcon_ws/
    $ colcon build --symlink-install
    $ source ~/colcon_ws/install/setup.sh
    ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- LAUNCH AND USAGE EXAMPLES -->
## Launch and Usage

1. Set the parameters inside [real_minimal.launch.py](sobit_home_bringup/launch/real_minimal.launch.py) and select the functions to launch with SOBIT HOME.
   ```python
    'enable_mobile_base' : 'True',
    'enable_arm_left'    : 'True',
    'enable_arm_right'   : 'True',
    'enable_head'        : 'True',
    ...
   ```
> [!NOTE]
> Rewrite it as `True` or `False` depending on the functions you want to use.


2. Execute the launch file [real_minimal.launch.py](sobit_home_bringup/launch/real_minimal.launch.py) in your **development environment**.
    ```sh
    $ ros2 launch sobit_home_bringup real_minimal.launch.py
    ```

If you did not succeed in connecting to Kachaka, check the following points:
- Ensure the emergency stop button is not pressed.
- Verify the battery is sufficiently charged.
- Confirm the USB hub is connected to the computer.
- Check if the Dynamixel Dongle is named `/dev/ttyUSB0`.
  - To verify, run `$ ls /dev` and if `/dev/ttyUSB1` is displayed, update the `usb_port` in [controllers.urdf.xacro](sobit_home_description/urdf/controllers.urdf.xacro).
- Ensure the Kachaka IP is correct.
- Verify that the `ROS_DOMAIN_ID` is the same on both the Kachaka and the development environment.

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

SOBIT HOME has a simulation environment with Gazebo Fortress, allowing you to verify operations even without the actual machine.

```sh
$ ros2 launch sobit_home_bringup gz_minimal.launch.py
```

At present, the following virtual environments are available.

| World Name   | Description |
| ------------ | ----------- |
| `empty`        | Spawns an environment without furniture or obstacles. |
| `wrs`          | Spawns the Tidy Up environment used in WRS2020. |
| `small_room`   | Spawns a small room layout developed by AWS. |

To change the environment, modify the `world_model` parameter in [gz_minimal.launch.py](sobit_home_bringup/launch/gz_minimal.launch.py).

<!-- If it works correctly, the following Gazebo screen will be displayed.
![SOBIT HOME Gazebo Harmonic](sobit_home/docs/img/sobit_home_gz_sim.png) -->

> [!TIP]
> Since it is equipped with sensors similar to the actual machine, the processing may become heavy depending on the computer. Please select only the necessary sensors in [gz_minimal.launch.py](sobit_home_bringup/launch/gz_minimal.launch.py).

```python
'enable_gz_head_cam_color'      : 'True',
'enable_gz_head_cam_depth'      : 'True',
'enable_gz_hand_left_cam_color' : 'True',
'enable_gz_hand_left_cam_depth' : 'True',
'enable_gz_hand_right_cam_color': 'True',
'enable_gz_hand_right_cam_depth': 'True',
'enable_gz_lidar_front'         : 'True',
'enable_gz_lidar_back'          : 'True',
```

Additionally, multiple SOBIT HOMEs can be spawned in the same simulation environment. To do so, configure [gz_minimal.launch.py](sobit_home_bringup/launch/gz_minimal.launch.py) to execute `gz_robot.launch.py` according to the number of robots.

Please, make sure that each `robot_name` have different values among robots.
Moreover, you can change the spawining coordinates of the robot in `robot_coords_x`, `robot_coords_y` and `robot_coords_z`.

Here is an example.
```python
...
# Launch Robot No. 1
IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            FindPackageShare('sobit_home_bringup'),
            'launch',
            'robot.launch.py'
        ])
    ]),
    launch_arguments={
        'robot_name': 'sobit_home_1',
        'robot_coords_x': '0', # x 
        'robot_coords_y': '0', # y
        'robot_coords_Y': '0', # yaw
        ...
    }.items()
),
# Launch Robot No. 2
IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            FindPackageShare('sobit_home_bringup'),
            'launch',
            'gz_robot.launch.py'
        ])
    ]),
    launch_arguments={
        'robot_name': 'sobit_home_2',
        'robot_coords_x': '0', # x 
        'robot_coords_y': '2', # y
        'robot_coords_Y': '0', # yaw
        ...
    }.items()
),
...
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## Software

<details>
<summary>Summary of information on SOBIT HOME and related software</summary>


### Joint Controller

This is a summary of information for moving the joints (pan-tilt mechanism, linear mechanism and manipulator) of SOBIT HOME.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


#### Movement Methods

<!-- 1. `move_to_pose` : Move it to a predetermined pose.
    ```yaml
    # MoveToPose.action
    # Goal
    string pose_name                                # Target pose name
    builtin_interfaces/Duration time_allowance      # Target time length
    ---
    # Result
    bool success                                    # Success / Failure
    string message                                  # Result message
    builtin_interfaces/Duration total_elapsed_time  # Finished time length
    ---
    # Feedback
    string[] current_joint_names                    # Currently moving joint name(s)
    float32[] current_joint_rad                     # Currently moving joint position(s)
    # float32[] current_joint_vel                   # Currently moving joint velocity(s)
    builtin_interfaces/Duration move_time           # Elapsed time length
    ```

> [!NOTE]
> Existing poses can be found in [pose_list.yaml](sobit_home_library/config/pose_list.yaml). Please refer to [How to set new poses](#how-to-set-new-poses) for how to create a new pose.

2. `move_joint` : Moves any joint to an arbitrary angle.
    ```yaml
    # MoveJoint.action
    # Goal
    string[] target_joint_names                     # Target joint name(s)
    float64[] target_joint_rad                      # Target joint position(s)
    builtin_interfaces/Duration time_allowance      # Target time length
    ---
    # Result
    bool success                                    # Success / Failure
    string message                                  # Result message
    builtin_interfaces/Duration total_elapsed_time  # Finished time length
    ---
    # Feedback
    string[] current_joint_names                    # Currently moving joint name(s)
    float64[] current_joint_rad                     # Currently moving joint position(s)
    # float32[] current_joint_vel                   # Currently moving joint velocity(s)
    builtin_interfaces/Duration move_time           # Elapsed time length
    ```

> [!NOTE]
> Please check the previously defined joint names in the [Joints Name](#joints-name) section.
 
3. `move_hand_to_target_coord` : Checks how to move the hand to the given xyz coordinates .
    ```yaml
    # MoveHandToTargetCoord.srv
    # Request
    geometry_msgs/TransformStamped target_coord     # Target coordinates

    ---
    # Result
    geometry_msgs/Pose move_pose                    # Moving pose for grasping
    string[] target_joint_names                     # List of joint names to move
    float64[] target_joint_rad                      # List of joint angles to move
    bool success                                    # Enable grasp
    string message                                  # Result message
    ```

4.  `move_hand_to_target_tf` : Checks how to move the hand to the given tf name.
    ```yaml
    # MoveHandToTargetTF.srv
    # Request
    string target_frame                             # Frame name to be grasped
    geometry_msgs/TransformStamped tf_differential  # Differential coordinates of Target frame
    ---
    # Result
    geometry_msgs/Pose move_pose                    # Moving pose for grasping
    string[] target_joint_names                     # List of joint names to move
    float64[] target_joint_rad                      # List of joint angles to move
    bool success                                    # Enable grasp
    string message                                  # Result message
    ``` -->

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
|  6 | arm_left_wrist_joint          | - |
|  7 | arm_right_shoulder_tilt_joint | - |
|  8 | arm_right_upper_roll_joint    | - |
|  9 | arm_right_upper_flex_joint    | - |
| 10 | arm_right_elbow_joint         | - |
| 11 | arm_right_wrist_joint         | - |
| 12 | hand_left_finger_mcp_joint    | - |
| 13 | hand_left_finger_l_cmc_joint  | - |
| 14 | hand_left_finger_l_mcp_joint  | - |
| 15 | hand_left_finger_c_cmc_joint  | - |
| 16 | hand_left_finger_c_mcp_joint  | - |
| 17 | hand_left_finger_r_cmc_joint  | - |
| 18 | hand_left_finger_r_mcp_joint  | - |
| 19 | hand_right_finger_mcp_joint   | - |
| 20 | hand_right_finger_l_cmc_joint | - |
| 21 | hand_right_finger_l_mcp_joint | - |
| 22 | hand_right_finger_c_cmc_joint | - |
| 23 | hand_right_finger_c_mcp_joint | - |
| 24 | hand_right_finger_r_cmc_joint | - |
| 25 | hand_right_finger_r_mcp_joint | - |
<!-- | 26 | wheel_steer_f_l_joint | - |
| 27 | wheel_steer_f_r_joint | - |
| 28 | wheel_steer_b_l_joint | - |
| 29 | wheel_steer_b_r_joint | - |
| 30 | wheel_drive_f_l_joint | - |
| 31 | wheel_drive_f_r_joint | - |
| 32 | wheel_drive_b_l_joint | - |
| 33 | wheel_drive_b_r_joint | - | -->

<p align="right">(<a href="#readme-top">back to top</a>)</p>


#### How to set new poses

Poses can be added and edited in the file [pose_list.yaml](sobit_home_library/config/pose_list.yaml). The format is as follows:

```yaml
poses:
    - initial_pose
    - detecting_pose
    - following_pose

initial_pose:
    head_pan                  : 0.0
    head_tilt                 : 0.0
    arm_left_shoulder_tilt    : 0.0
    arm_left_upper_roll       : 0.0
    arm_left_upper_flex       : 0.0
    arm_left_elbow            : 0.0
    arm_left_wrist            : 0.0
    arm_right_shoulder_tilt   : 0.0
    arm_right_upper_roll      : 0.0
    arm_right_upper_flex      : 0.0
    arm_right_elbow           : 0.0
    arm_right_wrist           : 0.0
...
```  

Add the desired pose name to `poses`, and then set the angles for each joint under the pose name.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Wheel Controller

This is a summary of information for moving the SOBIT HOME moving mechanism.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


#### Moving Methods

<!-- 1.  `move_wheel_linear` : Perform translational motion (straight-line only).
    ```yaml
    # MoveWheelLinear.action
    # Goal
    geometry_msgs/Point target_point                # Target Translational Distance
    builtin_interfaces/Duration time_allowance      # Target time length
    ---
    # Result
    bool success                                    # Success / Failure
    string message                                  # Result message
    builtin_interfaces/Duration total_elapsed_time  # Finished time length
    ---
    # Feedback
    geometry_msgs/Point current_point               # Currently displaced distance
    builtin_interfaces/Duration move_time           # Currently elapsed time
    ```  

2.  `move_wheel_rotate` : Perform rotational motion (units: Radian)
    ```yaml
    # MoveWheelRotate.action
    # Goal
    float32 target_yaw                              # Target Rotational Distance
    builtin_interfaces/Duration time_allowance      # Target time length
    ---
    # Result
    bool success                                    # Success / Failure
    string message                                  # Result message
    builtin_interfaces/Duration total_elapsed_time  # Finished time length
    ---
    # Feedback
    geometry_msgs/Point current_point               # Currently displaced distance
    builtin_interfaces/Duration move_time           # Currently elapsed time
    ``` -->

</details>

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## Hardware

SOBIT HOME is available as open hardware at [OnShape](https://cad.onshape.com/documents/).

<!-- ![SOBIT HOME in OnShape](sobit_home/docs/img/sobit_home_onshape.png) -->

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

<!-- ![SOBIT HOME Circuit](sobit_home/docs/img/sobit_home_circuit.svg) -->

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- ### Robot Assembly

TBD

<p align="right">(<a href="#readme-top">back to top</a>)</p> -->


### Features

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

<!-- | Part | Model Number | Quantity | Approx. Unit Cost | Where to Buy |
| --- | --- | --- | --- | --- |
| Kachaka | B1A01 | 1 | $1,569.45 | [link](https://store.kachaka.life/products/detail/50) |
| Kachaka Base | ksh0003 | 1 | $86.20 | [link](https://store.kachaka.life/products/detail/57) |
| Makita Battery | BL1860B | 1 | $181.58 | [link](https://www.makitatools.com/products/details/BL1860B) |
| Makita Adapter | B0D6R6XSPX | 1 | $25.90 | [link](https://www.amazon.com/dp/B0D6R6XSPX) |
| Dynamixel Actuators | XM430-W350-R | 6 | $318.89 | [link](https://www.robotis.us/dynamixel-xm430-w350-r/) |
| Dynamixel Actuators | XM540-W150-R | 4 | $472.89 | [link](https://www.robotis.us/dynamixel-xm540-w150-r/) |
| Dynamixel Frame | FR12-S102K Set | 2 | $20.90 | [link](https://www.robotis.us/fr12-s102k-set) |
| Dynamixel Frame | FR12-H101K Set | 1 |  $44.66 | [link](https://www.robotis.us/fr12-h101k-set/) |
| Dynamixel Frame | FR12-H104K Set | 1 | $41.91 | [link](https://www.robotis.us/fr12-h104k-set/) |
| Dynamixel Frame | FR13-H101K Set | 1 | $73.37 | [link](https://www.robotis.us/fr13-h101k-set/) |
| Dynamixel U2D2 | 8809052930103 | 1 | $35.31 | [link](https://www.robotis.us/u2d2/) |
| Dynamixel Power Hub | 8809052930530 | 1 | $35.31 | [link](https://www.robotis.us/u2d2-power-hub-board-set/) |
| (Optional) USB Hub | B0D1XVNTHJ | 1 | $23.99 | [link](https://www.amazon.com/dp/B0D1XVNTHJ) |
| (Optional) Speaker | Jabra Speak 710 | 1 | $233.90 | [link](https://www.jabra.com/business/speakerphones/jabra-speak-series/jabra-speak-710/) |
| (Optional) Microphone | MKE 400 | 1 | $196.87 | [link](https://www.sennheiser.com/en-ae/catalog/products/microphones/mke-400/mke-400-508898) |
| RealSense | D415 | 1 | $273.34 | [link](https://www.amazon.com/dp/B07JVGRQZT) |
| (Optional) RealSense | D405 | 1 | $278.12 | [link](https://www.amazon.com/dp/B09JBBHVTY) |
| (Optional) Stop Button | HW1B-X411R-MAU | 1 | $88.19 | [link](https://us.misumi-ec.com/vona2/detail/222000393180/?HissuCode=HW1B-X411R-MAU) |
| (Optional) M5Stack Basic V2.7 | K001-V27 | 1 | $39.90 | [link](https://shop.m5stack.com/products/esp32-basic-core-lot-development-kit-v2-7) |
| (Optional) ESP32 DevKitC-1-N16R8 | B0DWWY5KTZ | 1 | $9.99 | [link](https://www.amazon.com/dp/B0DWWY5KTZ) |
| (Optional) Display | B01CZL6QIQ | 2 | $14.49 | [link](https://www.amazon.com/dp/B01CZL6QIQ) |
| Thrust Roller Bearing | AXK1104 | 2 | $11.49 | [link](https://us.misumi-ec.com/vona2/detail/221000058345/?HissuCode=AXK1106) |
| Thrust Roller Bearing | AXK1106 | 1 | $9.08 | [link](https://us.misumi-ec.com/vona2/detail/221000058345/?HissuCode=AXK1106) |
| Aluminium Frame | HFS5-2020-600 | 1 | $9.42 | [link](https://us.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-600) |
| Aluminium Frame | HFS5-2020-100 | 6 | $4.66 | [link](https://us.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-100) |
| Aluminium Frame | HFS5-2020-110 | 1 | $4.66 | [link](https://us.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-110) |
| Brackets | HBLFSNK6 | 3 | $1.68 | [link](https://us.misumi-ec.com/vona2/detail/110300442520/?HissuCode=HBLFSNK6) |
| Socket Head Cap Screws | CSH-ST-M2-4 | 16 | $1.19 | [link](https://us.misumi-ec.com/vona2/detail/221000551286/?HissuCode=CSH-ST-M2-4) |
| Socket Head Cap Screws | CSH-ST-M2.5-5 | 54 | $0.39 | [link](https://us.misumi-ec.com/vona2/detail/221000551286/?HissuCode=CSH-ST-M2.5-5) |
| Socket Head Cap Screws | CSH-ST-M2.5-6 | 16 | $1.17 | [link](https://us.misumi-ec.com/vona2/detail/221000551286/?HissuCode=CSH-ST-M2.5-6) |
| Socket Head Cap Screws | CSH-ST-M2.5-8 | 34 | $0.73 | [link](https://us.misumi-ec.com/vona2/detail/221000551286/?HissuCode=CSH-ST-M2.5-8) |
| Socket Head Cap Screws | CSH-ST-M2.5-10 | 10 | $1.17 | [link](https://us.misumi-ec.com/vona2/detail/221000551286/?HissuCode=CSH-ST-M2.5-10) |
| Socket Head Cap Screws | CSH-ST-M2.5-12 | 16 | $1.17 | [link](https://us.misumi-ec.com/vona2/detail/221000551286/?HissuCode=CSH-ST-M2.5-12) |
| Socket Head Cap Screws | CSH-ST-M3-5 | 4 | $2.60 | [link](https://us.misumi-ec.com/vona2/detail/221000551286/?HissuCode=CSH-ST-M3-5) |
| Socket Head Cap Screws | CSH-ST-M4-15 | 16 | $1.16 | [link](https://us.misumi-ec.com/vona2/detail/221000551286/?HissuCode=CSH-ST-M4-15) |
| Socket Head Cap Screws | CSH-ST-M5-8 | 50 | $0.28 | [link](https://us.misumi-ec.com/vona2/detail/221000551286/?HissuCode=CSH-ST-M5-8) |
| Socket Head Cap Screws | CSH-ST-M5-12 | 12 | $0.29 | [link](https://us.misumi-ec.com/vona2/detail/221000551286/?HissuCode=CSH-ST-M5-12) |
| Socket Head Cap Screws | CSH-ST-M5-15 | 8 | $2.28 | [link](https://us.misumi-ec.com/vona2/detail/221000551286/?HissuCode=CSH-ST-M5-15) |
| Socket Head Cap Screws | CSH-ST-M5-20 | 4 | $3.77 | [link](https://us.misumi-ec.com/vona2/detail/221000551286/?HissuCode=CSH-ST-M5-20) |
| Socket Head Cap Screws | CSH-ST-M5-32 | 2 | $3.83 | [link](https://us.misumi-ec.com/vona2/detail/221000551286/?HissuCode=CSH-ST-M5-32) |
| Nut | LBNR2.5 | 24 | $0.16 | [link](https://us.misumi-ec.com/vona2/detail/110300250540/?HissuCode=LBNR2.5) |
| Nut | LBNR4 | 16 | $0.51 | [link](https://us.misumi-ec.com/vona2/detail/110300250540/?HissuCode=LBNR4) |
| Nut | LBNR5 | 26 | $0.51 | [link](https://us.misumi-ec.com/vona2/detail/110300250540/?HissuCode=LBNR5) |
| Nuts for 5 Series | HNTT5-5 | 44 | 0.63 | [link](https://us.misumi-ec.com/vona2/detail/110302246150/?HissuCode=HNTT5-5) |
| Power Adapter Plug Jack | B0BV8XCTC9 | 2 | $5.99 | [link](https://www.amazon.com/dp/B0BV8XCTC9) |
| eSUN Black Filament | ePLA+HS175B1KG-2SPOOL-US | 1 | $33.23 | [link](https://www.amazon.com/dp/B0D7Q1JYZM) |
| (Optional) eSUN Blue Filament |  ePLA+HS175U1KG-US  | 1 | $17.99 | [link](https://www.amazon.com/dp/B0CQT8VKF7) |

Total Approx. Cost (w/ Optional Items): **$7,510.47**

Total Approx. Cost (w/o Optional Items): **$6,592.54** -->

> [!IMPORTANT]
> Prices may vary depending on the retailer. Please check each link for the latest prices.

</details>

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- MILESTONE -->
## Milestone

- [] -

See the [open issues][issues-url] for a full list of proposed features (and known issues).

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
