# CAD analysis of the robot fish

`cad_analysis.py` estimates the physical properties of the robot fish from its CAD model (Onshape export: URDF + STL meshes) and the assumptions in `assumptions.yaml`. Its results are the source of the numbers used by the Unity simulation (`RobotFishPhysics` on `Assets/AUV/Prefabs/RobotFish.prefab`).

## What it computes

- Position of every part (forward kinematics of the URDF, all joints at 0), its volume, centroid and inertia (exact integration over the closed mesh).
- Mass budget: CAD volume × assumed density, or datasheet mass, for each part (`assumptions.yaml`).
- Displaced volume (hull envelope + parts outside the hull), centers of buoyancy and mass, trim ballast needed for the target buoyancy.
- Pitch authority of the sliding mass, inertia (with the added inertia of the water), tail and dorsal fin geometry.

The Onshape export has no materials: the URDF "mass" of each part is in fact its volume in m³. This is why the masses come from `assumptions.yaml`.

## Outputs

| File | Content |
| --- | --- |
| `Docs/RobotFish_CAD_Report.md` | all the results, including the values to use in Unity |
| `Docs/images/robot_fish_views.png` | side and top views of the CAD, with the centers of mass and buoyancy |

Both are generated: do not edit them by hand.

## Usage

The CAD is not stored in this repository (the meshes are large; `rpi_navigator.stl` alone is 73 MB). Download the Onshape export (folder `robot_fish/assembly` with `urdf/` and `meshes/`), then, from the repository root:

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

## Updating with real values

1. Edit `assumptions.yaml`: change the value, set `status: measured`.
2. Run the script again.
3. Copy the values of the section "Values for the Unity simulation" of the report into the `RobotFishPhysics` component of the `RobotFish` prefab (see `Docs/RobotFish_Assumptions.md` for the list of fields).

## Frames

- CAD: X lateral, Y long axis (front/dome at −Y, tail at +Y), Z up.
- Unity (local axes of the `RobotFish` prefab): forward = −X, up = +Y, right = +Z. Unity = (CAD y, CAD z, −CAD x).
