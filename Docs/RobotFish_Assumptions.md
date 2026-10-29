# Robot fish: assumptions to confirm

The simulation of the robot fish relies on values that were not measured on the real robot. This file lists all of them, with their source and where to change them, so that they can be replaced by real values later.

**Status:** *estimate* = assumed; *datasheet* = from a product sheet; *CAD* = measured on the CAD model; *measured* = measured on the real robot.

**Where to change:**
- `yaml` = `Tools/cad_analysis/assumptions.yaml`, then run `Tools/cad_analysis/cad_analysis.py` again (see its README) and copy the new values into Unity;
- `prefab` = component `RobotFishPhysics` of `Assets/AUV/Prefabs/RobotFish.prefab` (Unity Inspector).

## 1. What to measure first (largest effect on the simulation)

| # | Quantity | Current value | Why it matters | How to measure |
| --- | --- | --- | --- | --- |
| 1 | **Mass of the robot** (weighed in air, with everything inside except the moving mass) | 2.43 kg (estimate) | Decides whether it floats or sinks: with the estimate it floats strongly (+86 % with 100 g of moving mass) and needs ≈ 2.28 kg of moving mass to be neutral | Kitchen scale |
| 2 | Buoyancy in the basin, for a given moving mass | – | Confirms the displaced volume and the mass together | In the basin, note how much of the robot stays out of the water, or add weight until it barely floats |
| 3 | Weight of the moving mass in use | 100 g (team; adjustable) | Pitch authority and buoyancy | Weigh it |
| 4 | Longitudinal position of the center of mass of the structure | 39 mm behind the tube center (estimate: the tail servo is heavy and at the rear) | Decides where the moving mass must be for the robot to be level | Balance the robot (without the moving mass) on a rod across the tube |
| 5 | **Speed of the robot for a few flapping settings** (e.g. 1, 1.5 and 2 Hz at 30°) | simulated: 0.47, 0.72, 0.95 m/s | Calibrates the tail propulsion (added mass coefficient) and the hull drag together | Time it over a known distance in the basin, after the speed has settled |
| 6 | Roll and depth drift while flapping | simulated: ±4–5° of roll, slight climb at high speed | Checks the roll resonance found in the simulation (about 0.9 Hz, [RobotFish_Model.md](RobotFish_Model.md#51-tail-flapping)) | Video from the side and the front, IMU roll, pressure sensor |
| 7 | Whether the servo keeps the amplitude at high frequency | simulated: amplitude drops above ≈ 2 Hz at 30° | Servo torque and speed under the water load | Video from above, compare the real amplitude with the command |

## 2. Mass budget (`yaml`)

Each CAD part gets a mass from its CAD volume × an assumed density, or from a datasheet. The full table, with the resulting mass of each part, is in [RobotFish_CAD_Report.md](RobotFish_CAD_Report.md#mass-budget).

| Part(s) | Assumption | Status |
| --- | --- | --- |
| Tube, dome | acrylic, 1.19 g/cm³ | estimate |
| Caps, trays, clamp, servo mount, connectors, carriage (CAD), fins | 3D printed PLA, 1.24 g/cm³ (solid) | estimate |
| Rails | aluminium, 2.70 g/cm³ | estimate |
| Battery | LiPo, 2.2 g/cm³ (167 cm³ → 367 g) | estimate |
| Raspberry Pi 4 + Navigator | 80 g | estimate |
| Camera | 15 g | estimate |
| Servo HS-5496MH (CAD sliding mechanism) | 61 g | datasheet |
| Tail servo Hitec D955TW | 66 g + waterproof PLA housing filling the rest of its CAD volume at 60 % infill | servo: datasheet, housing: estimate |
| Rod (`haste`) | steel, 7.85 g/cm³ | estimate |
| Cables, screws, o-rings | 100 g at the tube center | estimate |
| Moving mass | 100 g on the tube axis (team: starts at 100 g, weight is added as needed); not part of the structure mass | team |
| Ballast | none: the robot carries only its parts and the moving mass (team) | team |

## 3. Geometry (CAD)

| Quantity | Value | Status | Where |
| --- | --- | --- | --- |
| Tube | Ø110 mm outer, Ø100 mm inner, 400 mm | CAD | prefab: `tubeRadius`, `tubeLength` |
| Caps (outside the tube) | Ø115.4 mm, 8 mm on each end | CAD | prefab: `capRadius`, `capThickness` |
| Dome | hemisphere Ø114.2 mm | CAD | prefab: `domeRadius` |
| Parts outside the hull (displace water) | clamp, servo mount, tail servo, arm, fins: 0.35 L | CAD | prefab: `externalVolumes` |
| Displaced volume | 4.71 L | CAD | computed by the prefab from the above |
| Tail servo axis | 325 mm behind the tube center | CAD | prefab: `tailHinge` |
| Tail fin | 191.5 cm², chord 160 mm, span 120 mm, center 17 mm below the axis | CAD | prefab: `tailFin.area`, `tailFinHeight` |
| Tail fin center of pressure | ¼ chord: 104 mm behind the servo axis | estimate (thin-plate theory) | prefab: `tailPressureDistance` |
| Dorsal fin | quarter disc R 75 mm, 42.1 cm² | CAD | prefab: `dorsalFin.area`, `dorsalFinCenter` |
| Sensor positions | camera in the dome, IMU on the Navigator board (CAD); pressure sensor on the rear cap and GNSS antenna 70 mm above the axis (**not in the CAD**, estimate) | CAD / estimate | prefab: children of `Sensors` |

## 4. Actuators

| Quantity | Value | Status | Where |
| --- | --- | --- | --- |
| Tail servo | Hitec D955TW at 7.4 V: 0.12 s/60° (500°/s), 29 kgf·cm | datasheet | prefab: `tailServoSpeed` |
| Tail servo torque | 2.84 N·m (29 kgf·cm), limits the acceleration of the fin and its water | datasheet | prefab: `tailServoTorque` |
| Tail servo position loop | gain 40 /s (how closely the servo follows the command) | estimate | prefab: `tailServoGain` |
| Inertia of the moving tail parts (hub, arm, fin), without water | 0.001 kg·m² about the servo axis | estimate | prefab: `tailFinInertia` |
| Tail fin angle | ±60° mechanical limit; rudder shortcut ±30° | team | yaml `tail_servo.max_angle_deg`, prefab: `maxTailAngle`, `rudderAngle` |
| Flapping range | up to 3 Hz (`/cmd/tail_frequency` = 1); cruise (Space, `/cmd/thrust`) 1.5 Hz with 30° | estimate | prefab: `maxTailFrequency`, `cruiseFrequency`, `cruiseAmplitude` |
| Moving mass | on the tube axis; travel 300 mm (tube length minus 50 mm at each end, approximation from the team), i.e. ±150 mm around the center; speed 50 mm/s (estimate, the drive is unknown); rest position computed so that the robot rests level | team / estimate | yaml `moving_mass`, prefab: `movingMass`, `movingMassTravel`, `movingMassSpeed`, `levelAtRest` |
| CAD sliding mechanism | HS-5496MH servo, horn 35 mm, rod 40 mm, carriage at the front of the tube: how it relates to the real moving mass is not known; static visual only | CAD | – |
| Propulsion | tail flapping (`TailFlapping`, default). Test model `SimpleThrust`: 0.3 N at the tail (about 0.3 m/s) | – | prefab: `propulsion`, `temporaryThrust` |
| Test ballast | ±25 % of the volume, simulation only (the real robot has none) | – | prefab: `ballastRange`, `ballastRate` |

## 5. Hydrodynamics

| Quantity | Value | Status | Where |
| --- | --- | --- | --- |
| Water density | 1000 kg/m³ (fresh water) | estimate | `RobotFishPhysics.WaterDensity` |
| Drag along the hull | C_D = 0.6 on the frontal area (95 cm²), includes the tail drive | estimate (literature) | prefab: `axialDragCoefficient` |
| Drag across the hull | C_D = 1.1 (cylinder in crossflow), 12 slices | estimate (literature) | prefab: `crossflowDragCoefficient`, `dragSlices` |
| Drag of the tail drive across | 70 cm² seen from the side, 40 cm² from above | estimate (CAD bounding boxes) | prefab: `tailDriveSideArea`, `tailDriveTopArea` |
| Fin lift slope | 1.2 /rad (aspect ratio 0.75, Helmbold formula); dorsal fin, and tail fin in the `SimpleThrust` model | estimate | prefab: `tailFin.liftSlope`, `dorsalFin.liftSlope` |
| Fin stall | 25°, then flat plate C_N = 1.2·sin α | estimate | prefab: `stallAngle` |
| Added inertia of the water | 0.0507 kg·m² in pitch and yaw (slender cylinder) | estimate | included in prefab `structureInertia` |
| Added mass of the flapping tail fin | 0.7 × the 2D flat-plate value ρ·π·span²/4 per unit chord (≈ 1.3 kg for the whole fin), for the 3D effects of the low aspect ratio | estimate (literature range 0.6–0.8) | prefab: `addedMassCoefficient` |
| Crossflow drag of the flapping fin | C_D = 1.2 (flat plate) | estimate (literature) | prefab: `finCrossflowDrag` |
| Fin discretization | 8 strips along the chord × 3 rows over the height | numerical | prefab: `tailFinStrips`, `tailFinRows` |
| Added inertia of the fin when the robot yaws | ≈ 0.27 kg·m², **not used** (`finAddedYawInertia` off, see [RobotFish_Model.md](RobotFish_Model.md#7-limitations)) | – | prefab: `finAddedYawInertia` |
| Added mass sideways / vertically | 3.8 kg, **not simulated** | – | – |
| Small linear damping | 0.1 N·s/m, 0.05 N·m·s | estimate (numerical) | prefab: `linearDamping`, `angularDamping` |
