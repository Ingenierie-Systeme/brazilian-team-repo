using UnityEngine;                                    // core Unity API
using Unity.Robotics.ROSTCPConnector;                 // ROSConnection
using RosMessageTypes.Sensor;                         // MagneticFieldMsg
using RosMessageTypes.Geometry;                       // Vector3Msg

// Magnetometer of the AUV (part of the INS), published as sensor_msgs/MagneticField [tesla].
// The Earth field is modeled as a constant vector in the world (intensity, inclination, declination);
// the sensor measures it in its own frame, plus a hard-iron bias and white noise.
// World convention (same as the GNSS): Unity +Z = north, +X = east, +Y = up.
public class MagnetometerPublisher : MonoBehaviour
{
    [Header("Earth magnetic field (default: Brest, approx. 2026 model values)")] // Inspector section title
    public float fieldIntensity = 47.9e-6f;           // total intensity [T] (~48 µT in Brest)
    public float inclinationDeg = 63.5f;              // dip angle below the horizon [degrees]
    public float declinationDeg = -1.0f;              // angle of magnetic north east of true north [degrees] (Brest: ≈ 1° west)

    [Header("Sensor")]                                // Inspector section title
    public float publishRate = 20f;                   // [Hz]
    public bool addNoise = true;                      // enable white noise
    public float noiseStd = 0.3e-6f;                  // noise per axis [T]
    public Vector3 hardIronBias = Vector3.zero;       // constant offset, ROS frame [T] (e.g. magnets of the servos)

    [Header("ROS 2")]                                 // Inspector section title
    public string topic = "imu/mag";                  // sensor_msgs/MagneticField topic
    public string frameId = "imu_link";               // same frame as the IMU

    [Header("Debug (read-only)")]                     // Inspector section title
    public Vector3 fieldMicroTesla;                   // last reading, ROS frame [µT]
    public float headingDeg;                          // heading computed from the reading (0 = magnetic north, clockwise) [degrees]
    public int messagesPublished;                     // messages sent

    private ROSConnection ros;                        // ROS connection
    private float timeSinceLastPublish;               // time since the last message [s]
    private readonly System.Random rng = new System.Random(); // noise generator

    // Registers the publisher
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();                                    // shared ROS connection
        ros.RegisterPublisher<MagneticFieldMsg>(topic);                               // magnetometer topic
    }

    // World field vector in Unity axes, from intensity, inclination and declination
    Vector3 WorldField()
    {
        float inc = inclinationDeg * Mathf.Deg2Rad;                                   // inclination [rad]
        float dec = declinationDeg * Mathf.Deg2Rad;                                   // declination [rad]
        float horizontal = fieldIntensity * Mathf.Cos(inc);                           // horizontal component [T]
        float down = fieldIntensity * Mathf.Sin(inc);                                 // downward component [T]
        return new Vector3(horizontal * Mathf.Sin(dec), -down, horizontal * Mathf.Cos(dec)); // (east, up, north) in Unity axes
    }

    // Publishes at the configured rate
    void Update()
    {
        timeSinceLastPublish += Time.deltaTime;                                       // rate control
        if (timeSinceLastPublish < 1f / publishRate) return;                          // not yet time to publish
        timeSinceLastPublish = 0f;                                                    // restart the counter

        Vector3 body = transform.InverseTransformDirection(WorldField());             // field seen by the sensor (Unity axes)
        Vector3 b = SensorUtils.ToRos(body) + hardIronBias;                           // to ROS FLU + hard-iron bias
        if (addNoise) b += SensorUtils.GaussianVector(rng, noiseStd);                 // white noise

        fieldMicroTesla = b * 1e6f;                                                   // debug, in µT
        headingDeg = Mathf.Repeat(Mathf.Atan2(b.y, b.x) * Mathf.Rad2Deg, 360f);       // level sensor: angle from x (forward) to magnetic north, clockwise
        double variance = addNoise ? (double)noiseStd * noiseStd : 0.0;               // per-axis variance
        ros.Publish(topic, new MagneticFieldMsg(SensorUtils.Header(frameId), new Vector3Msg(b.x, b.y, b.z), SensorUtils.Diagonal(variance)));
        messagesPublished++;                                                          // count
    }
}
