using UnityEngine;

// Turns on a bluish fog when the camera is below the water surface, inside the basin cavity.
[RequireComponent(typeof(Camera))]
public class UnderwaterEffect : MonoBehaviour
{
    public TestBasin basin;                           // basin (empty = search the scene)
    public Color fogColor = new Color(0.05f, 0.32f, 0.42f); // underwater fog color
    public float fogDensity = 0.12f;                  // fog density (exponential)
    public bool isUnderwater;                         // current state (read-only)

    // Fog settings before entering the water, restored when leaving it
    private bool savedFog;
    private Color savedColor;
    private FogMode savedMode;
    private float savedDensity;

    // Finds the basin if it was not set
    void Start()
    {
        if (basin == null) basin = FindAnyObjectByType<TestBasin>();
    }

    // Checks every frame whether the camera is under water
    void LateUpdate()
    {
        if (basin == null) return;
        Vector3 p = basin.transform.InverseTransformPoint(transform.position);
        bool under = Mathf.Abs(p.x) < basin.width / 2f
                  && Mathf.Abs(p.z) < basin.length / 2f
                  && p.y < basin.waterLevel && p.y > 0f;
        if (under != isUnderwater) SetUnderwater(under);
    }

    // Applies or restores the fog
    void SetUnderwater(bool value)
    {
        isUnderwater = value;
        if (value)
        {
            savedFog = RenderSettings.fog;
            savedColor = RenderSettings.fogColor;
            savedMode = RenderSettings.fogMode;
            savedDensity = RenderSettings.fogDensity;
            RenderSettings.fog = true;
            RenderSettings.fogMode = FogMode.Exponential;
            RenderSettings.fogColor = fogColor;
            RenderSettings.fogDensity = fogDensity;
        }
        else
        {
            RenderSettings.fog = savedFog;
            RenderSettings.fogColor = savedColor;
            RenderSettings.fogMode = savedMode;
            RenderSettings.fogDensity = savedDensity;
        }
    }

    // Restores the fog if the camera is disabled while under water
    void OnDisable()
    {
        if (isUnderwater) SetUnderwater(false);
    }
}
