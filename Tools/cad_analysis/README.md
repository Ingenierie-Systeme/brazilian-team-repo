# CAD tools of the robot fish

Two scripts work on the CAD model of the robot fish (Onshape export: URDF + STL meshes):

- `cad_analysis.py` estimates its physical properties (with the assumptions in `assumptions.yaml`). Its results are the source of the numbers used by the Unity simulation (`RobotFishPhysics`).
- `cad_to_unity.py` converts the CAD parts into meshes for the detailed Unity prefab (`Assets/AUV/Prefabs/RobotFishDetailed.prefab`).

The CAD is not stored in this repository (the meshes are large; `rpi_navigator.stl` alone is 73 MB): download the Onshape export (folder `robot_fish/assembly` with `urdf/` and `meshes/`) and give its path to the scripts.

# 1. CAD analysis (`cad_analysis.py`)

## 1.1 What it computes

- Position of every part (forward kinematics of the URDF, all joints at 0), its volume, centroid and inertia (exact integration over the closed mesh).
- Mass budget: CAD volume × assumed density, or datasheet mass, for each part (`assumptions.yaml`).
- Optional total mass of the structure given by the team (`structure_mass_target`): the difference with the parts is added as one mass at a given point. The moving mass can be set to `neutral` (`moving_mass.mass_g`): the script then uses the weight that makes the robot neutral.
- Displaced volume (hull envelope + parts outside the hull), centers of buoyancy and mass, buoyancy of the robot for several weights of the moving mass and the weight that makes it neutral (the robot carries no ballast).
- Pitch authority of the moving mass, inertia (with the added inertia of the water), tail and dorsal fin geometry.

The Onshape export has no materials: the URDF "mass" of each part is in fact its volume in m³. This is why the masses come from `assumptions.yaml`.

## 1.2 Outputs

| File | Content |
| --- | --- |
| `Docs/RobotFish_CAD_Report.md` | all the results, including the values to use in Unity |
| `Docs/images/robot_fish_views.png` | side and top views of the CAD, with the centers of mass and buoyancy |

Both are generated: do not edit them by hand.

## 1.3 Usage

From the repository root:

```bash
# once: Python dependencies (Ubuntu)
sudo apt install python3-numpy python3-yaml python3-matplotlib

# every time the CAD or the assumptions change
python3 Tools/cad_analysis/cad_analysis.py <path to>/robot_fish/assembly
```

Example with the CAD next to the repository:

```bash
python3 Tools/cad_analysis/cad_analysis.py ../CAD/robot_fish/assembly
```

Options: `--docs <folder>` (where the report and image go, default `Docs/`), `--assumptions <file>` (default `assumptions.yaml` next to the script).

## 1.4 Updating with real values

1. Edit `assumptions.yaml`: change the value, set `status: measured`.
2. Run the script again.
3. Copy the values of the section "Values for the Unity simulation" of the report into the `RobotFishPhysics` component of the simple `RobotFish` prefab (see `Docs/RobotFish_Assumptions.md` for the list of fields), then rebuild the detailed prefab (**Tools → Robot Fish → Build Detailed Prefab**), which copies its physics from the simple one.

# 2. CAD → Unity (`cad_to_unity.py`)

## 2.1 What it does

1. Places every part in the robot frame (same forward kinematics as `cad_analysis.py`) and converts it to the Unity frame of the robot.
2. Expresses the moving parts relative to their pivot: tail servo hub, arm and fin (turned back to 0°, the CAD was saved with the fin deflected 10.5°, relative to the servo axis). The pivots come from the URDF joints.
3. Simplifies the heavy meshes: quadric decimation (keeps the shape), then vertex clustering when decimation cannot go further (the Raspberry Pi board is made of thousands of tiny separate solids). 2.1 M → 255 k triangles.
4. Writes one OBJ file per part and `robot_fish_parts.json` (hierarchy, pivots, CAD colors, mechanism geometry) into `Assets/AUV/Models/RobotFish/`.

**Handedness:** the CAD frame is right-handed and Unity is left-handed. Unity's OBJ importer negates X on import, so the files are written in the right-handed frame (−X, Y, Z) of the final Unity frame.

## 2.2 Usage

The mesh simplification needs the `fast-simplification` package, installed once in a virtual environment (ignored by git):

```bash
# once, from the repository root
sudo apt install python3-venv python3-numpy python3-yaml
python3 -m venv --system-site-packages Tools/cad_analysis/.venv
Tools/cad_analysis/.venv/bin/pip install fast-simplification

# every time the CAD changes
Tools/cad_analysis/.venv/bin/python Tools/cad_analysis/cad_to_unity.py ../CAD/robot_fish/assembly
```

Then, in Unity: **Tools → Robot Fish → Build Detailed Prefab** (`Assets/AUV/Editor/RobotFishPrefabBuilder.cs`) rebuilds `RobotFishDetailed.prefab` from the converted meshes (physics and sensors are copied from the simple `RobotFish` prefab). **Tools → Robot Fish → Use Detailed Prefab In Basin Scenes** puts it in the basin scenes.

# 3. Frames

- CAD: X lateral, Y long axis (front/dome at −Y, tail at +Y), Z up.
- Unity (local axes of the `RobotFish` prefab): forward = −X, up = +Y, right = +Z. Unity = (CAD y, CAD z, −CAD x).
