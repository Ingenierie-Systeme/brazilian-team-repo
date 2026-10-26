# BrazilianTeamRepository

## Test basin simulation

Unity simulation of the 20 m × 12 m × 6 m test basin used to validate the micro-AUV.

![Test basin simulation](Docs/media/basin_simulation.gif)

- **Unity:** 6000.6.5f1, URP.
- **Scenes:** `Assets/Scenes/Basin_Grid.unity` (1 m metric grid, for visual debugging) and `Assets/Scenes/Basin_NoGrid.unity` (plain walls, closer to reality for vision algorithms). Both use the same `TestBasin` prefab and differ only by the basin material.
- **Features:** procedural basin, water volume with Archimedes buoyancy and drag, underwater fog, metric grid shader, and a ground-truth logger that writes the robot pose to CSV.

Open the project with Unity Hub (*Add project from disk*), load one of the scenes and press Play. See [Docs/TestBasin.md](Docs/TestBasin.md) for details.
