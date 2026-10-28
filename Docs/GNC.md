# Navigation, Guidance and Control (GNC)

High-level intelligence of the micro-AUV: heading following, depth regulation and, later, waypoint missions.
This document grows with each step of the GNC work; the current status is shown in section 2.

## 1. Architecture

```text
 sensors ──► NAVIGATION ──► estimated state ──► GUIDANCE ──► setpoints ──► CONTROL ──► actuator commands
 (Unity)     "where am I?"   depth, heading,     "where to     heading,      "how do I    /cmd/thrust, /cmd/rudder,
                             position, speed      go?"          depth, speed   get there?"  /cmd/pitch, /cmd/ballast
    ▲                                                                                              │
    └──────────────────────────────── Unity physics (SubmarinePhysics) ◄───────────────────────────┘
```

- **Navigation** estimates the state of the AUV from its sensors only (pressure, IMU, magnetometer, GNSS at the surface).
- **Guidance** turns the mission into setpoints (desired heading, depth and speed).
- **Control** computes the actuator commands that make the AUV follow the setpoints.

The simulation ground truth (`/ins/velocity`, `/gnss/ground_truth`, `GroundTruthLogger`) is never used by the GNC loop; it is only used to evaluate it.

Navigation, guidance and control run as ROS 2 nodes; the Unity simulation is the "plant" (physics + sensors + actuators).

## 2. Status

| Step | Content | Status |
| --- | --- | --- |
| 1 | Actuator command interface (Unity side) | done |
| 2 | Heading control | planned |
| 3 | Navigation node | planned |
| 4 | Depth control | planned |
| 5 | Guidance (setpoints, waypoints, mission) | planned |
| 6 | Launch file and evaluation | planned |

## 3. Actuator command interface

### 3.1 Topics

Each actuator has its own topic, type `std_msgs/Float64`, value normalized to [-1, 1]:

| Topic | Actuator | -1 | +1 | Physical maximum (`SubmarinePhysics`) |
| --- | --- | --- | --- | --- |
| `/cmd/thrust` | thruster | full reverse | full forward | `thrustForce` = 5 N |
| `/cmd/rudder` | rudder | full left turn | full right turn | `rudderTorque` = 0.2 N·m |
| `/cmd/pitch` | sliding mass (target position) | mass aft, nose up | mass forward, nose down | `maxPitchShift` = 3 cm |
| `/cmd/ballast` | ballast pump (speed) | empty, AUV lighter, rises | fill, AUV heavier, sinks | `ballastRate` = 0.25 /s (full range in 4 s) |

Values outside [-1, 1] are clamped; NaN is read as 0.

**Why normalized commands:** the controllers do not depend on the actuator sizes. When the real thrust or torque is measured, only the values in `SubmarinePhysics` change; the controllers keep working.

**Why one `Float64` topic per actuator:** no custom message to generate on the Unity and ROS sides, each actuator can be tested alone with `ros2 topic pub`, and each command can be plotted with `rqt_plot`.

**Ballast as a pump speed:** the AUV model treats the ballast as a pump that fills or empties the tank progressively, so the command is a speed and the fill level is its integral. The depth controller (step 4) takes this into account.

### 3.2 Unity side

- `SubmarinePhysics` (`Assets/AUV/Scripts/SubmarinePhysics.cs`) reads four public commands (`thrustCommand`, `rudderCommand`, `pitchCommand`, `ballastCommand`). With `externalControl` off, the keyboard writes them with the same keys and values as before (0 or ±1), so manual driving is unchanged. With `externalControl` on, the keyboard is ignored. The physics (buoyancy, damping, forces) is unchanged. The current ballast fill is available as `BallastLevel` (0 = empty, 1 = full).
- `ActuatorCommandSubscriber` (`Assets/AUV/Scripts/Control/ActuatorCommandSubscriber.cs`), on the `Submarine` prefab, subscribes to the four topics and hands the commands to `SubmarinePhysics` before each physics step. `SubmarinePhysics` itself has no dependency on ROS.

### 3.3 Watchdog

If no command arrives for `commandTimeout` = 0.5 s (controller stopped, endpoint down), the actuators go to a safe state (no thrust, rudder centered, mass centered, pump stopped) and the keyboard takes over again. A controller must therefore publish continuously (typically 10–20 Hz), not only when the command changes.

### 3.4 Manual test

**Terminal 1: ROS-TCP-Endpoint** (workspace with the endpoint, see [AUV_Sensors.md](AUV_Sensors.md#5-running-with-ros-2) to create it once):

```bash
source /opt/ros/jazzy/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST   # see the note on shared networks below
cd ~/ros2_ws && source install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1
```

**Unity:** open `Assets/TestBasin/Scenes/Basin_Grid.unity`, press Play and click the Game view. The ROS indicator (top left) turns blue when connected.

**Terminal 2: commands.** `ros2 topic pub` keeps publishing until Ctrl+C, so run one command at a time (or each one in its own terminal to combine them):

```bash
source /opt/ros/jazzy/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST   # see the note on shared networks below

# moves forward at full thrust (about 0.4 m/s); Ctrl+C to stop
ros2 topic pub -r 10 /cmd/thrust std_msgs/msg/Float64 "{data: 1.0}"

# turns left at half rudder torque; Ctrl+C to stop
ros2 topic pub -r 10 /cmd/rudder std_msgs/msg/Float64 "{data: -0.5}"

# empties the ballast: the AUV rises to the surface; Ctrl+C to stop
ros2 topic pub -r 10 /cmd/ballast std_msgs/msg/Float64 "{data: -1.0}"
```

After Ctrl+C the AUV stops within 0.5 s and the keyboard works again (watchdog).

**Optional, terminal 3: watch the result** while a command is published:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST   # see the note on shared networks below
ros2 topic echo /depth                                     # depth decreases while the ballast is emptied
ros2 topic echo /imu/data --field angular_velocity.z       # yaw rate, positive while turning left
```

In the Unity Inspector, `Submarine` → `Actuator Command Subscriber` shows `Ros In Control`, the last commands received and the message count.

**Note on shared networks (e.g. the lab network):** by default ROS 2 discovers every machine of the local network, so `ros2 topic list` also shows the topics of other robots, and commands such as `/cmd/thrust` could reach them (or theirs could reach the simulation). `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` keeps the ROS graph on this machine, which is enough because Unity talks to the endpoint over TCP on 127.0.0.1. Set it in every terminal, including the endpoint one. After changing it, run `ros2 daemon stop` once so that the ROS 2 daemon forgets the robots it already discovered.

### 3.5 Verification

Scripted test in `Basin_Grid`: a ROS 2 node published a command sequence at 20 Hz, Unity logged the AUV state.

| Commands | Result |
| --- | --- |
| none | AUV at rest at y = 3.00 m, keyboard control |
| thrust 1 | speed 0.38 m/s after ~1.5 s (≈ 6° nose-down trim, as with the keyboard) |
| thrust 1, rudder 0.5 | yaw rate 0.26 rad/s (≈ half of the full-rudder rate) |
| nothing for 2 s | watchdog after 0.5 s, back to keyboard, AUV stops |
| thrust -1 | moves backwards at 0.38 m/s |
| thrust 1, pitch 1 | nose down, the AUV dives (y 2.50 → 2.06 m in 2 s) |
| ballast -1 | ballast 0.50 → 0 in 2 s, the AUV rises to the surface (y = 5.41 m) |
