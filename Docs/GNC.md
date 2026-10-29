# Navigation, Guidance and Control (GNC)

High-level intelligence of the robot fish: heading following, depth regulation and, later, waypoint missions.
This document grows with each step of the GNC work; the current status is shown in section 2.

## 1. Architecture

```text
 sensors ──► NAVIGATION ──► estimated state ──► GUIDANCE ──► setpoints ──► CONTROL ──► actuator commands
 (Unity)     "where am I?"   depth, heading,     "where to     heading,      "how do I    /cmd/tail_frequency,
                             position, speed      go?"          depth, speed   get there?"  /cmd/tail_amplitude,
                                                                                            /cmd/tail_bias, /cmd/pitch
    ▲                                                                                              │
    └──────────────────────────────── Unity physics (RobotFishPhysics) ◄───────────────────────────┘
```

- **Navigation** estimates the state of the AUV from its sensors only (pressure, IMU, magnetometer, GNSS at the surface).
- **Guidance** turns the mission into setpoints (desired heading, depth and speed).
- **Control** computes the actuator commands that make the AUV follow the setpoints.

The simulation ground truth (`/ins/velocity`, `/gnss/ground_truth`, `GroundTruthLogger`) is never used by the GNC loop; it is only used to evaluate it.

Navigation, guidance and control run as ROS 2 nodes; the Unity simulation is the "plant" (physics + sensors + actuators).

**The real robot is a robot fish** ([RobotFish_Model.md](RobotFish_Model.md)): no propeller and no ballast. A single tail fin, moved side to side by a servo, gives propulsion (by flapping) and steering; it changes depth only by swimming forward with the nose tilted by the moving mass that slides along the tube axis (aft = nose up, forward = nose down), like a glider. The simulation keeps a ballast to set up tests, but the GNC never uses it. See section 4.

## 2. Status

| Step | Content | Status |
| --- | --- | --- |
| 1 | Actuator command interface (Unity side) | done |
| 1b | Robot fish model from the CAD: mass, buoyancy, water drag, moving mass, tail and dorsal fins ([RobotFish_Model.md](RobotFish_Model.md)) | done |
| 1c | Tail flapping propulsion and tail commands (frequency, amplitude, bias) ([RobotFish_Model.md](RobotFish_Model.md#35-tail-flapping-propulsion)) | done |
| 2 | Heading control | planned |
| 3 | Navigation node | planned |
| 4 | Depth control | planned |
| 5 | Guidance (setpoints, waypoints, mission) | planned |
| 6 | Launch file and evaluation | planned |

## 3. Actuator command interface

### 3.1 Topics

Each actuator has its own topic, type `std_msgs/Float64`, value normalized:

| Topic | Actuator | Range | Physical maximum (`RobotFishPhysics`) |
| --- | --- | --- | --- |
| `/cmd/tail_frequency` | flapping frequency of the tail fin | 0 … 1 | `maxTailFrequency` = 3 Hz |
| `/cmd/tail_amplitude` | flapping amplitude | 0 … 1 | `maxTailAngle` = 60° |
| `/cmd/tail_bias` | mean fin angle of the flapping: −1 full left turn, +1 full right turn | −1 … 1 | `maxTailAngle` = 60° |
| `/cmd/thrust` | shortcut: swim at cruise (flapping 1.5 Hz, amplitude thrust × 30°); negative values do nothing | 0 … 1 | `cruiseFrequency`, `cruiseAmplitude` |
| `/cmd/rudder` | shortcut: fin bias, −1 full left, +1 full right (turns only while swimming) | −1 … 1 | `rudderAngle` = 30° |
| `/cmd/pitch` | moving mass on the tube axis: −1 back end (nose up), +1 front end (nose down) | −1 … 1 | 0 = rest position (robot level); `movingMassTravel` = ±150 mm |
| `/cmd/ballast` | test ballast (speed), **simulation only**: −1 lighter (rises), +1 heavier (sinks) | −1 … 1 | `ballastRange` = ±25 % of the volume, `ballastRate` = 12.5 %/s |

Values outside the range are clamped; NaN is read as 0. The shortcuts `/cmd/thrust` and `/cmd/rudder` (also driven by the keyboard) are used only while the three tail commands are 0; the GNC controllers use the tail commands. With the `SimpleThrust` test model, `/cmd/thrust` is a forward force (0.3 N, ≈ 0.3 m/s, −1 = reverse) and `/cmd/rudder` a fixed fin angle (±30°).

**Why normalized commands:** the controllers do not depend on the actuator sizes. When the real actuators are measured, only the values in `RobotFishPhysics` change; the controllers keep working.

**Why one `Float64` topic per actuator:** no custom message to generate on the Unity and ROS sides, each actuator can be tested alone with `ros2 topic pub`, and each command can be plotted with `rqt_plot`.

**Ballast, simulation only:** the real robot has no ballast, so the GNC never publishes on `/cmd/ballast`. The topic only helps to set up tests in the simulation (e.g. bring the robot to a given depth before a test). The command is a pump speed; the buoyancy change is its integral.

### 3.2 Unity side

- `RobotFishPhysics` (`Assets/AUV/Scripts/RobotFishPhysics.cs`) reads public commands (`tailFrequencyCommand`, `tailAmplitudeCommand`, `tailBiasCommand`, `thrustCommand`, `rudderCommand`, `pitchCommand`, `ballastCommand`). With `externalControl` off, the keyboard writes the shortcuts and the others (Space, A/D, W/S, Q/E: 0 or ±1). With `externalControl` on, the keyboard is ignored.
- `ActuatorCommandSubscriber` (`Assets/AUV/Scripts/Control/ActuatorCommandSubscriber.cs`), on the robot fish prefabs, subscribes to the seven topics and hands the commands to `RobotFishPhysics` before each physics step. `RobotFishPhysics` itself has no dependency on ROS.

### 3.3 Watchdog

If no command arrives for `commandTimeout` = 0.5 s (controller stopped, endpoint down), the actuators go to a safe state (no flapping, fin centered, mass centered, pump stopped) and the keyboard takes over again. A controller must therefore publish continuously (typically 10–20 Hz), not only when the command changes.

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

# swims forward at cruise (flapping 1.5 Hz, 30°: about 0.7 m/s); Ctrl+C to stop
ros2 topic pub -r 10 /cmd/thrust std_msgs/msg/Float64 "{data: 1.0}"

# or set the flapping directly: 2 Hz (needs an amplitude too, in another terminal); Ctrl+C to stop
ros2 topic pub -r 10 /cmd/tail_frequency std_msgs/msg/Float64 "{data: 0.667}"

# amplitude 30° (0.5 × 60°), with the frequency of the previous command; Ctrl+C to stop
ros2 topic pub -r 10 /cmd/tail_amplitude std_msgs/msg/Float64 "{data: 0.5}"

# test ballast, simulation only: the robot gets lighter and rises; Ctrl+C to stop
ros2 topic pub -r 10 /cmd/ballast std_msgs/msg/Float64 "{data: -1.0}"
```

**Turning needs forward speed** (section 4): keep the thrust command of terminal 2 running and, in **terminal 3**, shift the flapping to one side:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST   # see the note on shared networks below

# turns left with a 15° bias, while terminal 2 publishes the thrust; Ctrl+C to stop
ros2 topic pub -r 10 /cmd/rudder std_msgs/msg/Float64 "{data: -0.5}"

# with the direct tail commands of terminal 2, use the bias instead: 10° to the left; Ctrl+C to stop
ros2 topic pub -r 10 /cmd/tail_bias std_msgs/msg/Float64 "{data: -0.17}"
```

After Ctrl+C the actuators return to neutral within 0.5 s and the keyboard works again (watchdog).

**Optional, terminal 4: watch the result** while a command is published:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST   # see the note on shared networks below
ros2 topic echo /depth                                     # depth decreases while the ballast is emptied
ros2 topic echo /imu/data --field angular_velocity.z       # yaw rate, positive while turning left
```

In the Unity Inspector, `RobotFishDetailed` → `Actuator Command Subscriber` shows `Ros In Control`, the last commands received and the message count.

**Note on shared networks (e.g. the lab network):** by default ROS 2 discovers every machine of the local network, so `ros2 topic list` also shows the topics of other robots, and commands such as `/cmd/thrust` could reach them (or theirs could reach the simulation). `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` keeps the ROS graph on this machine, which is enough because Unity talks to the endpoint over TCP on 127.0.0.1. Set it in every terminal, including the endpoint one. After changing it, run `ros2 daemon stop` once so that the ROS 2 daemon forgets the robots it already discovered.

### 3.5 Verification

Scripted test in `Basin_Grid`: a ROS 2 node published a command sequence at 20 Hz, Unity logged the AUV state. This test was made with the first AUV model (propeller and rudder torque); the robot fish model is verified in [RobotFish_Model.md](RobotFish_Model.md#5-verification).

| Commands | Result |
| --- | --- |
| none | AUV at rest at y = 3.00 m, keyboard control |
| thrust 1 | speed 0.38 m/s after ~1.5 s (≈ 6° nose-down trim, as with the keyboard) |
| thrust 1, rudder 0.5 | yaw rate 0.26 rad/s (≈ half of the full-rudder rate) |
| nothing for 2 s | watchdog after 0.5 s, back to keyboard, AUV stops |
| thrust -1 | moves backwards at 0.38 m/s |
| thrust 1, pitch 1 | nose down, the AUV dives (y 2.50 → 2.06 m in 2 s) |
| ballast -1 | ballast 0.50 → 0 in 2 s, the AUV rises to the surface (y = 5.41 m) |

## 4. What the GNC must take into account

The physics of the robot fish is described in [RobotFish_Model.md](RobotFish_Model.md). For the GNC, the main consequences are:

- **No ballast:** depth is controlled only through the pitch (moving mass) while swimming forward. Vertical speed ≈ forward speed × sin(pitch). With the moving mass heavy enough to be neutral (≈ 2.3 kg), a few centimetres of displacement tilt the robot by tens of degrees and its full travel (±150 mm) makes it vertical, so the depth controller must limit the displacement it commands. Depth control will be a cascade: depth error → desired pitch → moving-mass position.
- **Weak self-righting:** the center of mass is only 3–6 mm below the center of buoyancy, so the robot tilts and rolls easily and pitch oscillates slowly after a change; the pitch loop must damp it.
- **One fin for propulsion and steering:** speed is set by the flapping frequency and amplitude (about 0.7 m/s at 1.5 Hz and 30°, see [RobotFish_Model.md](RobotFish_Model.md#51-tail-flapping)), the heading by the bias of the oscillation (10° of bias: 12°/s while swimming at cruise). The two are coupled: a large bias reduces the useful amplitude (the fin is limited to ±60°) and the speed.
- **Turning needs speed:** the bias only steers while the robot swims.
- **The heading oscillates with each stroke** (±6° to ±14° depending on the amplitude): the heading controller must not try to cancel this oscillation, it should act on the heading averaged over a flapping period (low-pass filter or averaging on the magnetometer/IMU heading).
- **Servo limits:** above about 2 Hz at 30° the servo can no longer follow the commanded amplitude, and beyond 2.5 Hz the water moment exceeds its stall torque. The controllers should stay within this range.
- **Roll and depth with flapping:** the flapping makes the robot roll a few degrees and, at high speed, climb slowly (the thrust acts below the center of mass); the depth controller compensates with the moving mass.
- **Unknown buoyancy:** it depends on the real masses and on the weight added to the moving mass (with 100 g the robot floats strongly). The controllers must work for a neutral robot and for a slightly floating one.
