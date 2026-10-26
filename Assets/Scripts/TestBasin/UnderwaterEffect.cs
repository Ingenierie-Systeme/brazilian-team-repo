using UnityEngine;                                    // core Unity API

// Turns on a bluish fog when the camera is below the water surface, inside the basin cavity.
[RequireComponent(typeof(Camera))]                    // the effect belongs to a camera
public class UnderwaterEffect : MonoBehaviour
{
    public TestBasin basin;                           // basin (empty = search the scene)
    public Color fogColor = new Color(0.05f, 0.32f, 0.42f); // underwater fog color
    public float fogDensity = 0.12f;                  // fog density (exponential)
    public bool isUnderwater;                         // current state (read-only)

    private bool savedFog;                            // was the original fog on?
    private Color savedColor;                         // original fog color
    private FogMode savedMode;                        // original fog mode
    private float savedDensity;                       // original fog density

    // Finds the basin if it was not set
    void Start()
    {
        if (basin == null) basin = FindAnyObjectByType<TestBasin>(); // look for a basin in the scene
    }

    // Checks every frame whether the camera is under water
    void LateUpdate()
    {
        if (basin == null) return;                                      // no basin: nothing to do
        Vector3 p = basin.transform.InverseTransformPoint(transform.position); // camera position in basin space
        bool under = Mathf.Abs(p.x) < basin.width / 2f                 // within the width
                  && Mathf.Abs(p.z) < basin.length / 2f                // within the length
                  && p.y < basin.waterLevel && p.y > 0f;               // between the floor and the surface
        if (under != isUnderwater) SetUnderwater(under);                // switch only when the state changes
    }

    // Applies or restores the fog
    void SetUnderwater(bool value)
    {
        isUnderwater = value;                                           // update the state
        if (value)                                                      // entering the water
        {
            savedFog = RenderSettings.fog;                              // save the original setting
            savedColor = RenderSettings.fogColor;                       // save the original color
            savedMode = RenderSettings.fogMode;                         // save the original mode
            savedDensity = RenderSettings.fogDensity;                   // save the original density
            RenderSettings.fog = true;                                  // turn the fog on
            RenderSettings.fogMode = FogMode.Exponential;               // exponential fog (similar to water)
            RenderSettings.fogColor = fogColor;                         // bluish color
            RenderSettings.fogDensity = fogDensity;                     // underwater density
        }
        else                                                            // leaving the water
        {
            RenderSettings.fog = savedFog;                              // restore the original fog
            RenderSettings.fogColor = savedColor;                       // restore the color
            RenderSettings.fogMode = savedMode;                         // restore the mode
            RenderSettings.fogDensity = savedDensity;                   // restore the density
        }
    }

    // Restores the fog if the camera is disabled while under water
    void OnDisable()
    {
        if (isUnderwater) SetUnderwater(false);                         // back to normal
    }
}
