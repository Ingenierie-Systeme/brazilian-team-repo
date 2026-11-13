using System;
using System.Globalization;                          // CultureInfo (decimal point in the CSV)
using System.IO;
using UnityEngine;
using UnityEngine.SceneManagement;

// Writes the robot's real pose (ground truth) and the distance travelled on the XZ plane to a CSV file,
// to be compared with the pose estimated by the robot (odometry, SLAM...).
public class GroundTruthLogger : MonoBehaviour
{
    [Header("Sampling")]
    public float sampleRate = 20f;                   // samples per second [Hz]
    public bool logOnStart = true;                   // start recording when Play starts

    [Header("File")]
    public string outputFolder = "";                 // output folder (empty = <project>/GroundTruthLogs in the editor)
    public string filePrefix = "groundtruth";        // file name prefix

    [Header("Debug (read-only)")]
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
        if (logOnStart) StartLogging();
    }

    // Creates the CSV file and writes the header
    [ContextMenu("Start logging")]
    public void StartLogging()
    {
        StopLogging();
        string folder = string.IsNullOrEmpty(outputFolder)
            ? (Application.isEditor ? Path.Combine(Application.dataPath, "..", "GroundTruthLogs") // editor: next to Assets
                                    : Path.Combine(Application.persistentDataPath, "GroundTruthLogs")) // build: user data folder
            : outputFolder;
        Directory.CreateDirectory(folder);
        string scene = SceneManager.GetActiveScene().name;                   // scene name (tells grid / no-grid runs apart)
        string stamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");             // date/time so files are not overwritten
        currentFile = Path.GetFullPath(Path.Combine(folder, $"{filePrefix}_{scene}_{stamp}.csv"));
        writer = new StreamWriter(currentFile, false);
        writer.WriteLine("time_s,pos_x,pos_y,pos_z,rot_x,rot_y,rot_z,rot_w,euler_x_deg,euler_y_deg,euler_z_deg,distance_xz_m");
        lastPosition = transform.position;
        distanceXZ = 0f;
        samplesWritten = 0;
        startTime = Time.time;
        nextSampleTime = Time.time;
    }

    // Accumulates the distance at every physics step and writes samples at the configured rate
    void FixedUpdate()
    {
        if (writer == null) return;

        Vector3 p = transform.position;
        distanceXZ += new Vector2(p.x - lastPosition.x, p.z - lastPosition.z).magnitude;
        lastPosition = p;

        if (Time.time < nextSampleTime) return;
        nextSampleTime += 1f / Mathf.Max(sampleRate, 0.1f);

        Quaternion q = transform.rotation;
        Vector3 e = q.eulerAngles;
        CultureInfo inv = CultureInfo.InvariantCulture;                      // force the dot as decimal separator
        writer.WriteLine(string.Join(",",
            (Time.time - startTime).ToString("F4", inv),
            p.x.ToString("F4", inv), p.y.ToString("F4", inv), p.z.ToString("F4", inv),
            q.x.ToString("F6", inv), q.y.ToString("F6", inv), q.z.ToString("F6", inv), q.w.ToString("F6", inv),
            e.x.ToString("F3", inv), e.y.ToString("F3", inv), e.z.ToString("F3", inv),
            distanceXZ.ToString("F4", inv)));
        samplesWritten++;
        if (samplesWritten % 50 == 0) writer.Flush();                        // write to disk from time to time
    }

    // Closes the file safely
    [ContextMenu("Stop logging")]
    public void StopLogging()
    {
        if (writer == null) return;
        writer.Flush();
        writer.Dispose();
        writer = null;
        Debug.Log($"GroundTruthLogger: {samplesWritten} samples written to {currentFile}");
    }

    // Closes the file when the component is disabled or Play mode stops
    void OnDisable()
    {
        StopLogging();
    }
}
