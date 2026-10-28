# BrazilianTeamRepository

## Test basin simulation

Unity simulation of the 20 m × 12 m × 6 m test basin used to validate the robot fish.

![Test basin simulation](Docs/media/basin_simulation.gif)

- **Unity:** 6000.6.5f1, URP.
- **Scenes:** `Assets/TestBasin/Scenes/Basin_Grid.unity` (1 m metric grid, for visual debugging) and `Assets/TestBasin/Scenes/Basin_NoGrid.unity` (plain walls, closer to reality for vision algorithms). Both use the same `TestBasin` prefab and differ only by the basin material.
- **Features:** procedural basin, water volume, underwater fog, metric grid shader, the robot fish (`Assets/AUV/`) swimming inside the basin, and a ground-truth logger that writes the robot pose to CSV.
- **Controls:** Space = temporary thrust, A/D = tail fin (turns only while moving), W/S = pitch (sliding mass), Q/E = test ballast (simulation only), C = free camera (right mouse + WASD).

Open the project with Unity Hub (*Add project from disk*), load one of the scenes and press Play. See [Docs/TestBasin.md](Docs/TestBasin.md) for details.

## Robot fish

The real robot is a robot fish: no propeller and no ballast. A tail fin moved side to side by a servo gives propulsion and steering, and a sliding weight tilts it so that it changes depth while swimming forward. The simulation model (`RobotFish` prefab) is built from the CAD: mass budget, buoyancy over the real displaced volume, water drag, sliding mass and fins.

![Robot fish in the simulation](Docs/images/robot_fish_unity.png)

- [Docs/RobotFish_Model.md](Docs/RobotFish_Model.md): the robot and its physical model, tests.
- [Docs/RobotFish_Assumptions.md](Docs/RobotFish_Assumptions.md): every value that is still an estimate, and what to measure on the real robot.
- [Docs/RobotFish_CAD_Report.md](Docs/RobotFish_CAD_Report.md): properties computed from the CAD by [Tools/cad_analysis](Tools/cad_analysis/README.md).

## AUV sensors and ROS 2

The robot carries an RGB camera, an INS (gyroscope, accelerometer, magnetometer), a pressure sensor and a GNSS receiver (fix only at the surface). They publish through the ROS-TCP-Connector:

| Topic                        | Message                                             |
| ---------------------------- | --------------------------------------------------- |
| `/camera/compressed_image` | `sensor_msgs/CompressedImage` (JPEG)              |
| `/imu/data`, `/imu/mag`  | `sensor_msgs/Imu`, `sensor_msgs/MagneticField`  |
| `/pressure`, `/depth`    | `sensor_msgs/FluidPressure`, `std_msgs/Float64` |
| `/gnss/fix`                | `sensor_msgs/NavSatFix`                           |

Start the [ROS-TCP-Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint/tree/main-ros2) (`ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1`), then press Play. See [Docs/AUV_Sensors.md](Docs/AUV_Sensors.md).

## Navigation, guidance and control

The robot actuators can be commanded from ROS 2 (`std_msgs/Float64`, normalized to [-1, 1]):

| Topic            | Actuator                                 |
| ---------------- | ---------------------------------------- |
| `/cmd/thrust`  | temporary thrust until the tail flapping model (+ forward) |
| `/cmd/rudder`  | tail fin (+ turn right; needs forward speed) |
| `/cmd/pitch`   | sliding mass (+ nose down)               |
| `/cmd/ballast` | test ballast, simulation only (+ heavier/sink, - lighter/rise) |

The real robot has no ballast, so `/cmd/ballast` only helps to set up simulation tests. Without commands for 0.5 s the actuators stop and the keyboard takes over again. See [Docs/GNC.md](Docs/GNC.md) for the architecture and the status of the GNC work.

## Project structure

Unity assets are grouped by feature:

```text
Assets/
├── TestBasin/   test basin simulation (scripts, grid shader, materials, prefab, scenes)
├── AUV/         robot fish (physics, sensors, ROS control, prefab) and the original AUV scene
└── Settings/    URP render pipeline settings
Docs/            documentation, images and media
Tools/
└── cad_analysis/  physical properties of the robot from its CAD (Python)
```
