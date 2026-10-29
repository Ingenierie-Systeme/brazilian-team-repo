using System.Collections.Generic;                     // List
using UnityEngine;                                    // core Unity API
using UnityEngine.InputSystem;                        // keyboard (new Input System)

// Presentation flight of the camera through the campus scene: a smooth path through a list of waypoints
// (Catmull-Rom spline for the position, interpolated look target), from the parking lot into the building and
// down to the robot. While the tour runs, SimulationCamera is disabled; at the end it takes over again
// (follow mode), so the robot can be driven right away.
//   T: start / stop the tour.
[RequireComponent(typeof(Camera))]
public class CameraTour : MonoBehaviour
{
    [System.Serializable]
    public struct Waypoint
    {
        public Vector3 position;                      // camera position [m, world]
        public Vector3 lookAt;                        // point the camera looks at [m, world]
        public float duration;                        // time to reach the next waypoint [s]
    }

    [Header("Tour")]
    public List<Waypoint> waypoints = new List<Waypoint>();
    [Tooltip("Start the tour when Play starts.")]
    public bool playOnStart = true;
    public Key toggleKey = Key.T;
    [Tooltip("Camera that takes over at the end of the tour (empty = search on this object).")]
    public SimulationCamera simulationCamera;

    [Header("Debug (read-only)")]
    public bool playing;
    public float tourTime;                            // time since the tour started [s]

    void Start()
    {
        if (simulationCamera == null) simulationCamera = GetComponent<SimulationCamera>();
        if (playOnStart) Play();
    }

    public void Play()
    {
        if (waypoints.Count < 2) return;
        playing = true;
        tourTime = 0f;
        if (simulationCamera != null) simulationCamera.enabled = false;
    }

    public void Stop()
    {
        playing = false;
        if (simulationCamera != null) simulationCamera.enabled = true;
    }

    void Update()
    {
        if (Keyboard.current != null && Keyboard.current[toggleKey].wasPressedThisFrame)
        {
            if (playing) Stop(); else Play();
        }
    }

    void LateUpdate()
    {
        if (!playing) return;
        tourTime += Time.deltaTime;

        // find the segment i → i+1 that contains tourTime
        float t = tourTime;
        int i = 0;
        while (i < waypoints.Count - 1 && t > waypoints[i].duration)
        {
            t -= waypoints[i].duration;
            i++;
        }
        if (i >= waypoints.Count - 1) { Stop(); return; }
        float u = Mathf.Clamp01(t / Mathf.Max(waypoints[i].duration, 0.01f));
        u = u * u * (3f - 2f * u);                    // ease in/out on each segment

        Vector3 p0 = waypoints[Mathf.Max(i - 1, 0)].position;
        Vector3 p1 = waypoints[i].position;
        Vector3 p2 = waypoints[i + 1].position;
        Vector3 p3 = waypoints[Mathf.Min(i + 2, waypoints.Count - 1)].position;
        transform.position = CatmullRom(p0, p1, p2, p3, u);
        Vector3 look = Vector3.Lerp(waypoints[i].lookAt, waypoints[i + 1].lookAt, u);
        transform.rotation = Quaternion.LookRotation(look - transform.position, Vector3.up);
    }

    // Catmull-Rom spline: passes through p1 (u = 0) and p2 (u = 1), with tangents from the neighbours
    static Vector3 CatmullRom(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float u)
    {
        float u2 = u * u, u3 = u2 * u;
        return 0.5f * (2f * p1 + (p2 - p0) * u + (2f * p0 - 5f * p1 + 4f * p2 - p3) * u2 + (3f * p1 - p0 - 3f * p2 + p3) * u3);
    }
}
