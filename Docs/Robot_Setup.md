# Running the real robot

How to install, calibrate and start the code on the Raspberry Pi of the robot fish. **It has only been tested on a computer, with a fake board.** The first tests on the robot will likely show things to adjust.

## 1. Architecture

```text
 Navigator board ◄──► navigator ──/imu/data, /imu/mag──► fusion ──/state/attitude──► navigation ──/state/heading──► heading_control
 Bar30 ────────────► barometer ──/depth─────────────────────────────────────────────►                                     │
                                                                                     /cmd/tail_frequency, amplitude, bias │
 tail servo    ◄──── navigator ◄──/servo/tail── tail_oscillator ◄─────────────────────────────────────────────────────────┘
 moving mass   ◄──── navigator ◄──/cmd/pitch── depth_control or manual command
```

Package `ros2_ws/src/auv_hardware`, based on the `navigator` package of the course. A single node talks to the Navigator board. The topics above the servos are the same as in the simulator, so the GNC nodes do not change.

| Safety | Behavior |
| --- | --- |
| Start | servos at the center before the outputs are enabled |
| No command for 0.5 s | each servo returns to the center |
| Node stopped (Ctrl+C, SIGTERM) | servos at the center, then outputs disabled |
| Command out of range | limited to ±1, NaN read as 0 |
| A sensor stops | no estimate, no control command, so the tail stops |
| `robot.launch.py` without `swim:=true` | the tail does not flap |

The camera is not part of this package yet: no GNC node uses it.

## 2. Installation (once)

Follow `rpi_setup.md` of the course (Ubuntu 24.04, ROS 2 Jazzy, `bluerobotics_navigator`, `imufusion`, `scipy`, `python3-smbus2`). Then, on the robot:

```bash
source /opt/ros/jazzy/setup.bash
cd <path to>/brazilian-team-repo/ros2_ws
colcon build --symlink-install --packages-select gnc_control auv_hardware
colcon test --packages-select gnc_control auv_hardware
colcon test-result --verbose
```

## 3. During the assembly

Before fitting the servo hub, center the servo, as asked in the servo presentation of the course. In terminal 1, start the driver, which holds both servos at the center:

```bash
source /opt/ros/jazzy/setup.bash
cd <path to>/brazilian-team-repo/ros2_ws && source install/setup.bash
ros2 run auv_hardware navigator --ros-args --params-file src/auv_hardware/config/robot.yaml
```

## 4. Checks, in this order

Fin free, out of the water. All values are written in `ros2_ws/src/auv_hardware/config/robot.yaml`; with `--symlink-install`, restart the nodes to apply a change, no new build is needed.

**Terminal 1: driver** (as in section 3).

**Terminal 2: one command at a time, Ctrl+C between them.** Start with small values:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic pub -r 10 /servo/tail std_msgs/msg/Float64 "{data: 0.2}"    # tail servo
ros2 topic pub -r 10 /cmd/pitch std_msgs/msg/Float64 "{data: 0.2}"     # moving-mass servo
```

| # | Check | If not | Parameter |
| --- | --- | --- | --- |
| 1 | The tail servo moves with `/servo/tail`, the mass with `/cmd/pitch` | Swap the channels | `tail_channel`, `mass_channel` |
| 2 | Each servo returns to the center 0.5 s after Ctrl+C | Stop and check the driver | – |
| 3 | A positive `/cmd/pitch` moves the mass forward | Reverse it | `mass_invert` |
| 4 | The mass does not hit anything at ±1 (increase slowly) | Limit it, and note the real travel | `mass_limit` |
| 5 | A positive `/servo/tail` moves the fin to the right, seen from above with the nose forward | Reverse it | `invert` of `tail_oscillator` |
| 6 | Measure the fin angle at `/servo/tail` = 1 (increase slowly) | – | `servo_range_deg` |

## 5. Calibration (required)

Without calibration, the heading control fails ([GNC.md, section 7.4](GNC.md#74-results-simulation-closed-loop-through-ros-2)). Stop the driver and use the `imu_calib` tools of the course, with the robot assembled and on, away from metal:

```bash
python3 tool_calib-gyro.py                 # 10 s without moving: gyro_offset
python3 tool_calib-accel.py                # six faces: accel_offset, accel_matrix
python3 tool_calib-mag-log.py              # 75 s turning in all directions: writes mag-log.npy
python3 tool_calib-mag-fit.py mag-log.npy  # mag_soft (M) and mag_hard (v)
```

Write the results in the section `fusion` of `robot.yaml`. Then start the nodes, without flapping (the launch file starts the driver, so stop any other copy first). The robot must be at the surface, because this pressure is the zero of the depth.

**Terminal 1: robot:**

```bash
source /opt/ros/jazzy/setup.bash
cd <path to>/brazilian-team-repo/ros2_ws && source install/setup.bash
ros2 launch auv_hardware robot.launch.py
```

**Terminal 2: state** (one command at a time, Ctrl+C between them):

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /state/heading
ros2 topic echo /state/roll
ros2 topic echo /state/pitch
ros2 topic echo /state/depth
```

| # | Check | If not | Parameter |
| --- | --- | --- | --- |
| 1 | Nose down: pitch positive. Right side down: roll positive | Change the axes | `axes` of `fusion` |
| 2 | Turn the robot to the right: the heading grows, and it matches a compass within a few degrees | Change the axes, redo the magnetometer calibration | `axes`, `mag_hard`, `mag_soft` |
| 3 | Turn by 90° quickly: the heading follows without a slow correction afterward | Check the gyroscope scale | `gyro_scale` |
| 4 | Lower the sensor by a known depth: `/state/depth` follows | Check the density | `water_density` |

## 6. First swim

**Out of the water first, fin free.** The launch file with `swim:=true` starts the flapping as soon as the estimate is ready, about 3 s after the start. It starts at 15° of amplitude and holds the heading measured at that moment.

**Terminal 1: robot:**

```bash
source /opt/ros/jazzy/setup.bash
cd <path to>/brazilian-team-repo/ros2_ws && source install/setup.bash
ros2 launch auv_hardware robot.launch.py swim:=true
```

**Terminal 2: heading setpoint** (+ = clockwise; keeps publishing until Ctrl+C):

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic pub -r 2 /guidance/heading std_msgs/msg/Float64 "{data: 90.0}"
```

**Terminal 3: result:**

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /gnc/heading_error
```

Check that the fin bias goes to the side of the error (setpoint to the right, fin to the right). Ctrl+C in terminal 1 centers the servos and disables the outputs.

**In the water:** the robot depends on a computer connection to start and stop, and Wi-Fi does not reach under water, so stay at the surface for the first tests, with a way to recover the robot.

## 7. Depth control

The robot needs to swim to change depth. Before the depth control:

1. **Measure BG:** let the robot roll in the water and time the period T. Then BG ≈ 4π²·I / (M·g·T²), with I the roll inertia, about 0.006 kg·m² in the simulation. Write it in `bg_m` of `depth_control`.
2. **Measure the travel of the moving mass** at `/cmd/pitch` = ±1, and write it in `moving_mass_travel_m`.
3. **Weigh the robot,** and write `total_mass_kg` and `moving_mass_kg`.
4. **Check if the travel is enough:** a travel Δx gives a pitch at rest of about tan(pitch) = m·Δx / (M·BG). In the simulation, about ±7° at the end of the travel was not enough and ±20° worked; values between them were not tested ([GNC.md, section 9.3](GNC.md#93-results-simulation-closed-loop-through-ros-2)).

`robot.launch.py swim:=true` also starts the depth control, which holds the depth measured at start. To give a depth (the limits of `robot.yaml` start at 2 m and ±10° of pitch):

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic pub -r 2 /guidance/depth std_msgs/msg/Float64 "{data: 1.0}"
```

## 8. Mission and teleop

**Tie a rope to the robot.** Radio does not cross the water, so the teleop only works while the connection reaches it.

The first mission should be `timed.yaml`, which does not use the position, with a delay of 30 s to put the robot in the water. Check the headings and depths of the file first.

**Terminal 1, on the robot:**

```bash
source /opt/ros/jazzy/setup.bash
cd <path to>/brazilian-team-repo/ros2_ws && source install/setup.bash
ros2 launch auv_hardware robot.launch.py swim:=true mission:=timed.yaml
```

**Terminal 2, on the robot, to take it at any moment** (any control key takes it, X stops everything, R releases it; the mission does not resume):

```bash
source /opt/ros/jazzy/setup.bash
cd <path to>/brazilian-team-repo/ros2_ws && source install/setup.bash
ros2 run gnc_control teleop
```

**Terminal 3, the state of the mission:**

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /mission/status
```

Terminal 1 keeps running if the connection is lost, as long as it runs in `tmux` or `screen` on the robot. Start it that way before a test in the water. Test the takeover out of the water first.

## 9. Calibrating the position

The position comes from a speed table measured in the simulation ([GNC.md, section 8](GNC.md#8-step-3b-position-by-dead-reckoning)). Time the robot over a known straight distance in the water, once the speed has settled, at the amplitude in use. Then write in `speed_scale` of `robot.yaml`: real speed / speed of the table (0.36 m/s at 1.5 Hz and 15°).

## 10. What to record, if there is time

A video from above and the logs of the roll and heading, at 1.5 Hz with 15° and 30° of amplitude, to compare with the simulation (±3.2° and ±6.4° of yaw, [RobotFish_Model.md](RobotFish_Model.md#51-tail-flapping)):

```bash
source /opt/ros/jazzy/setup.bash
ros2 bag record /imu/data /imu/mag /depth /state/heading /state/roll /state/pitch /servo/tail
```
