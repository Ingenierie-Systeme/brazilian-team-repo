using UnityEngine;                                    // core Unity API

// Keeps SubmarinePhysics.waterLevel equal to the water surface height of the TestBasin,
// so the water height is defined in a single place (the basin) and the AUV script stays unchanged.
[RequireComponent(typeof(SubmarinePhysics))]          // only makes sense on the submarine
public class SubmarineWaterLevelSync : MonoBehaviour
{
    public TestBasin basin;                           // basin providing the water height (empty = search the scene)

    private SubmarinePhysics submarine;               // AUV physics component on the same object

    // Finds the components and applies the water level before the first physics step
    void Start()
    {
        submarine = GetComponent<SubmarinePhysics>(); // AUV physics on this object
        if (basin == null) basin = FindAnyObjectByType<TestBasin>(); // look for a basin in the scene
        if (basin == null) Debug.LogWarning("SubmarineWaterLevelSync: no TestBasin in the scene", this); // warn if none
        Sync();                                       // first update
    }

    // Updates the water level every physics step (follows the basin if it is moved or edited)
    void FixedUpdate()
    {
        Sync();                                       // keep both values equal
    }

    // Copies the basin water surface height into the AUV physics
    void Sync()
    {
        if (basin != null && submarine != null) submarine.waterLevel = basin.WaterSurfaceY; // world Y of the surface
    }
}
