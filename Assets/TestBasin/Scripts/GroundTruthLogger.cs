using System;                                        // DateTime
using System.Globalization;                          // CultureInfo (decimal point in the CSV)
using System.IO;                                     // files and folders
using UnityEngine;                                   // core Unity API
using UnityEngine.SceneManagement;                   // active scene name

// Writes the robot's real pose (ground truth) and the distance travelled on the XZ plane to a CSV file,
// to be compared with the pose estimated by the robot (odometry, SLAM...).
public class GroundTruthLogger : MonoBehaviour
{
    [Header("Sampling")]                             // Inspector section title
    public float sampleRate = 20f;                   // samples per second [Hz]
    public bool logOnStart = true;                   // start recording when Play starts

    [Header("File")]                                 // Inspector section title
    public string outputFolder = "";                 // output folder (empty = <project>/GroundTruthLogs in the editor)
    public string filePrefix = "groundtruth";        // file name prefix

    [Header("Debug (read-only)")]                    // Inspector section title
    public float distanceXZ;                         // accumulated distance on the XZ plane [m]
    public int samplesWritten;                       // lines written
    public string currentFile;                       // current file path

    private StreamWriter writer;                     // CSV writer
    private Vector3 lastPosition;                    // position at the previous step (for the distance)
    private float startTime;                         // time when recording started
    private float nextSampleTime;                    // time of the next sample

    // Opens the file at start, if enabled
    void Start()
    {
        if (logOnStart) StartLogging();              // start recording automatically
    }

    // Creates the CSV file and writes the header
    [ContextMenu("Start logging")]                   // item in the component's ⋮ menu
    public void StartLogging()
    {
        StopLogging();                                                       // close a previous file, if any
        string folder = string.IsNullOrEmpty(outputFolder)                   // chosen or default folder
            ? (Application.isEditor ? Path.Combine(Application.dataPath, "..", "GroundTruthLogs") // editor: next to Assets
                                    : Path.Combine(Application.persistentDataPath, "GroundTruthLogs")) // build: user data folder
            : outputFolder;                                                  // path set in the Inspector
        Directory.CreateDirectory(folder);                                   // create the folder if it does not exist
        string scene = SceneManager.GetActiveScene().name;                   // scene name (tells grid / no-grid runs apart)
        string stamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");             // date/time so files are not overwritten
        currentFile = Path.GetFullPath(Path.Combine(folder, $"{filePrefix}_{scene}_{stamp}.csv")); // full path
        writer = new StreamWriter(currentFile, false);                       // open the file for writing
        writer.WriteLine("time_s,pos_x,pos_y,pos_z,rot_x,rot_y,rot_z,rot_w,euler_x_deg,euler_y_deg,euler_z_deg,distance_xz_m"); // header
        lastPosition = transform.position;                                   // starting point of the distance
        distanceXZ = 0f;                                                     // reset the distance
        samplesWritten = 0;                                                  // reset the counter
        startTime = Time.time;                                               // mark the start
        nextSampleTime = Time.time;                                          // first sample immediately
    }

    // Accumulates the distance at every physics step and writes samples at the configured rate
    void FixedUpdate()
    {
        if (writer == null) return;                                          // not recording

        Vector3 p = transform.position;                                      // current position
        distanceXZ += new Vector2(p.x - lastPosition.x, p.z - lastPosition.z).magnitude; // add the horizontal segment
        lastPosition = p;                                                    // keep for the next step

        if (Time.time < nextSampleTime) return;                              // not yet time to sample
        nextSampleTime += 1f / Mathf.Max(sampleRate, 0.1f);                  // schedule the next sample

        Quaternion q = transform.rotation;                                   // current rotation
        Vector3 e = q.eulerAngles;                                           // Euler angles (degrees)
        CultureInfo inv = CultureInfo.InvariantCulture;                      // force the dot as decimal separator
        writer.WriteLine(string.Join(",",                                    // build the CSV line
            (Time.time - startTime).ToString("F4", inv),                     // time since start [s]
            p.x.ToString("F4", inv), p.y.ToString("F4", inv), p.z.ToString("F4", inv), // position [m]
            q.x.ToString("F6", inv), q.y.ToString("F6", inv), q.z.ToString("F6", inv), q.w.ToString("F6", inv), // quaternion
            e.x.ToString("F3", inv), e.y.ToString("F3", inv), e.z.ToString("F3", inv), // Euler [degrees]
            distanceXZ.ToString("F4", inv)));                                // accumulated distance [m]
        samplesWritten++;                                                    // count the line
        if (samplesWritten % 50 == 0) writer.Flush();                        // write to disk from time to time
    }

    // Closes the file safely
    [ContextMenu("Stop logging")]                                            // item in the component's ⋮ menu
    public void StopLogging()
    {
        if (writer == null) return;                                          // nothing open
        writer.Flush();                                                      // flush the buffer
        writer.Dispose();                                                    // close the file
        writer = null;                                                       // mark as closed
        Debug.Log($"GroundTruthLogger: {samplesWritten} samples written to {currentFile}"); // tell where the file is
    }

    // Closes the file when the component is disabled or Play mode stops
    void OnDisable()
    {
        StopLogging();                                                       // make sure the file is closed
    }
}
