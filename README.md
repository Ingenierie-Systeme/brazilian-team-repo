# BrazilianTeamRepository

## Test basin simulation

Unity simulation of the 20 m × 12 m × 6 m test basin used to validate the robot fish.

![Test basin simulation](Docs/media/basin_simulation.gif)

- **Unity:** 6000.6.5f1, URP.
- **Scenes:** `Assets/TestBasin/Scenes/Basin_Grid.unity` (1 m metric grid, for visual debugging) and `Assets/TestBasin/Scenes/Basin_NoGrid.unity` (plain walls, closer to reality for vision algorithms). Both use the same `TestBasin` prefab and differ only by the basin material.
- **Features:** procedural basin, water volume, underwater fog, metric grid shader, the robot fish (`Assets/AUV/`) swimming inside the basin, and a ground-truth logger that writes the robot pose to CSV.
- **Controls:** Space = swim (tail flapping), A/D = turn (while swimming), W/S = pitch (moving mass), Q/E = test ballast (simulation only), C = free camera (right mouse + WASD).

Open the project with Unity Hub (*Add project from disk*), load one of the scenes and press Play. See [Docs/TestBasin.md](Docs/TestBasin.md) for details.

## Campus scene

Presentation scene: the test basin and the robot fish inside a model of the ENSTA Air-Sea Test Basin building (Brest campus), with the parking lot, trees, a sky and a camera tour from the campus down to the robot (`Assets/Environment/Scenes/AirSeaTestBasin_Campus.unity`, T = tour). Sensors and ROS 2 commands work as in the basin scenes. See [Docs/Campus_Scene.md](Docs/Campus_Scene.md).

![Campus scene](Docs/images/campus_aerial.png)

## Robot fish

The real robot is a robot fish: no propeller and no ballast. A tail fin flapped side to side by a servo gives propulsion and steering (simulated with the reactive forces of the water on the fin, about 0.7 m/s at 1.5 Hz), and a moving mass sliding along the tube axis tilts it so that it changes depth while swimming forward. The simulation model is built from the CAD: mass budget, buoyancy over the real displaced volume, water drag, moving mass and fins. The basin scenes use `RobotFishDetailed`, made of the CAD parts with a transparent hull, whose moving mass and tail move with the commands (a lightweight `RobotFish` prefab with simple shapes has the same physics).

![Detailed robot fish in the simulation](Docs/images/robot_fish_detailed_side.png)

- [Docs/RobotFish_Model.md](Docs/RobotFish_Model.md): the robot and its physical model, tests.
- [Docs/RobotFish_Assumptions.md](Docs/RobotFish_Assumptions.md): every value that is still an estimate, and what to measure on the real robot.
- [Docs/RobotFish_CAD_Report.md](Docs/RobotFish_CAD_Report.md): properties computed from the CAD by [Tools/cad_analysis](Tools/cad_analysis/README.md), which also converts the CAD parts for Unity.

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

The robot actuators can be commanded from ROS 2 (`std_msgs/Float64`, normalized values):

| Topic            | Actuator                                 |
| ---------------- | ---------------------------------------- |
| `/cmd/tail_frequency` | flapping frequency of the tail fin (0 … 1 × 3 Hz) |
| `/cmd/tail_amplitude` | flapping amplitude (0 … 1 × 60°) |
| `/cmd/tail_bias` | mean fin angle (−1 … 1 × 60°, + turn right; needs forward speed) |
| `/cmd/thrust`, `/cmd/rudder` | shortcuts: swim at cruise (1.5 Hz, 30°), turn with a 30° bias |
| `/cmd/pitch`   | moving mass (+ forward, nose down)       |
| `/cmd/ballast` | test ballast, simulation only (+ heavier/sink, - lighter/rise) |

The real robot has no ballast, so `/cmd/ballast` only helps to set up simulation tests. Without commands for 0.5 s the actuators stop and the keyboard takes over again. See [Docs/GNC.md](Docs/GNC.md) for the architecture and the status of the GNC work.

## Project structure

Unity assets are grouped by feature:

```text
Assets/
├── TestBasin/   test basin simulation (scripts, grid shader, materials, prefab, scenes)
├── AUV/         robot fish (physics, sensors, ROS control, CAD models, prefabs, prefab builder) and the original AUV scene
├── Environment/ campus scene: Air-Sea Test Basin building and surroundings (scene builder, camera tour, materials, textures)
└── Settings/    URP render pipeline settings
Docs/            documentation, images and media
Tools/
├── cad_analysis/  physical properties of the robot from its CAD, CAD → Unity conversion (Python)
└── scene_assets/  logo and sign textures of the campus scene (Python)
```
