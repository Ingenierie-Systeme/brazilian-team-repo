using System;                                         // Math
using UnityEngine;                                    // core Unity API
using Unity.Robotics.ROSTCPConnector;                 // ROSConnection
using RosMessageTypes.Sensor;                         // NavSatFixMsg, NavSatStatusMsg
using RosMessageTypes.Geometry;                       // PointStampedMsg, PointMsg

// GNSS receiver of the AUV, published as sensor_msgs/NavSatFix on /gnss/fix.
// GNSS signals do not go through water: with the antenna submerged the receiver reports STATUS_NO_FIX.
// Unity world → local ENU (East = X, North = Z, Up = Y) → latitude/longitude/altitude around a reference
// point, with a local tangent-plane approximation (cm accuracy over a few km).
// Noise: first-order Gauss-Markov (slowly drifting error, like a real receiver) plus a little white noise.
public class GnssPublisher : MonoBehaviour
{
    [Header("Sensor")]                                // Inspector section title
    public TestBasin basin;                           // basin providing the water surface (empty = search the scene)
    public float publishRate = 5f;                    // typical receivers: 1–10 Hz
    public float maxAntennaDepth = 0.05f;             // antenna depth still accepted as "at the surface" [m]

    [Header("Reference (Unity origin)")]              // Inspector section title
    public double referenceLatitude = 48.7128;        // [deg] (approx. ENSTA campus, Palaiseau)
    public double referenceLongitude = 2.2190;        // [deg]
    public double referenceAltitude = 160.0;          // [m] above the WGS84 ellipsoid

    [Header("Noise")]                                 // Inspector section title
    public bool addNoise = true;                      // enable the position error
    public float horizontalStd = 2.0f;                // [m] typical standalone receiver: 2–5 m
    public float verticalStd = 4.0f;                  // [m] usually about twice the horizontal error
    public float errorCorrelationTime = 30f;          // how slowly the error drifts [s] (0 = white noise only)
    [Range(0f, 1f)] public float whiteNoiseFraction = 0.2f; // part of the std that is white noise

    [Header("ROS 2")]                                 // Inspector section title
    public string fixTopic = "gnss/fix";              // sensor_msgs/NavSatFix topic
    public bool publishLocal = true;                  // also publish ENU positions in meters (easy to plot)
    public string localTopic = "gnss/local";          // measured ENU position (geometry_msgs/PointStamped)
    public string groundTruthTopic = "gnss/ground_truth"; // true ENU position (simulation only)
    public string frameId = "gnss_link";              // frame name in the message header

    [Header("Debug (read-only)")]                     // Inspector section title
    public bool hasFix;                               // antenna above the water?
    public Vector3 trueEnu;                           // true position [m]
    public Vector3 measuredEnu;                       // measured position [m]
    public int fixesPublished;                        // messages with a fix

    const double A = 6378137.0;                       // WGS84 semi-major axis [m]
    const double E2 = 6.69437999014e-3;               // WGS84 first eccentricity squared

    private ROSConnection ros;                        // ROS connection
    private float timeSinceLastFix;                   // time since the last message [s]
    private Vector3 correlatedError;                  // slowly drifting error, ENU [m]
    private readonly System.Random rng = new System.Random(); // noise generator

    // Finds the basin, registers the publishers and initializes the error
    void Start()
    {
        if (basin == null) basin = FindAnyObjectByType<TestBasin>();                  // look for a basin in the scene
        ros = ROSConnection.GetOrCreateInstance();                                    // shared ROS connection
        ros.RegisterPublisher<NavSatFixMsg>(fixTopic);                                // fix topic
        if (publishLocal)                                                             // local ENU topics
        {
            ros.RegisterPublisher<PointStampedMsg>(localTopic);                       // measured
            ros.RegisterPublisher<PointStampedMsg>(groundTruthTopic);                 // true
        }
        if (addNoise) correlatedError = ScaledGaussian(CorrelatedFraction);           // start with a settled error
    }

    // Publishes at the configured rate
    void Update()
    {
        timeSinceLastFix += Time.deltaTime;                                           // rate control
        if (timeSinceLastFix < 1f / publishRate) return;                              // not yet time to publish
        float dt = timeSinceLastFix;                                                  // real elapsed time
        timeSinceLastFix = 0f;                                                        // restart the counter

        Vector3 p = transform.position;                                               // antenna position (Unity)
        trueEnu = new Vector3(p.x, p.z, p.y);                                         // Unity → ENU
        float surface = basin != null ? basin.WaterSurfaceY : float.NegativeInfinity; // water surface height
        hasFix = p.y > surface - maxAntennaDepth;                                     // antenna (almost) out of the water?
        var header = SensorUtils.Header(frameId);                                     // message header

        if (!hasFix)                                                                  // submerged: no satellite signal
        {
            ros.Publish(fixTopic, new NavSatFixMsg(header,
                new NavSatStatusMsg(NavSatStatusMsg.STATUS_NO_FIX, NavSatStatusMsg.SERVICE_GPS), // no fix
                double.NaN, double.NaN, double.NaN,                                   // no position
                new double[9], NavSatFixMsg.COVARIANCE_TYPE_UNKNOWN));                // no covariance
            return;                                                                   // nothing else to publish
        }

        measuredEnu = trueEnu;                                                        // start from the true position
        if (addNoise)                                                                 // GNSS error
        {
            UpdateCorrelatedError(dt);                                                // slow drift
            measuredEnu += correlatedError + ScaledGaussian(whiteNoiseFraction);      // + white noise
        }
        EnuToGeodetic(measuredEnu, out double lat, out double lon, out double alt);   // ENU → latitude/longitude/altitude

        double h2 = horizontalStd * horizontalStd, v2 = verticalStd * verticalStd;    // variances [m²]
        ros.Publish(fixTopic, new NavSatFixMsg(header,
            new NavSatStatusMsg(NavSatStatusMsg.STATUS_FIX, NavSatStatusMsg.SERVICE_GPS), // valid fix
            lat, lon, alt,                                                            // position
            new[] { h2, 0, 0, 0, h2, 0, 0, 0, v2 },                                   // ENU covariance, row-major
            NavSatFixMsg.COVARIANCE_TYPE_DIAGONAL_KNOWN));                            // diagonal known
        if (publishLocal)                                                             // ENU in meters
        {
            ros.Publish(localTopic, new PointStampedMsg(header, new PointMsg(measuredEnu.x, measuredEnu.y, measuredEnu.z))); // measured
            ros.Publish(groundTruthTopic, new PointStampedMsg(header, new PointMsg(trueEnu.x, trueEnu.y, trueEnu.z)));       // true
        }
        fixesPublished++;                                                             // count
    }

    // Local tangent plane: lat = lat0 + North/M, lon = lon0 + East/(N·cos lat0), alt = alt0 + Up
    void EnuToGeodetic(Vector3 enu, out double lat, out double lon, out double alt)
    {
        double lat0 = referenceLatitude * Math.PI / 180.0;                            // reference latitude [rad]
        double s = Math.Sin(lat0);                                                    // sin(lat0)
        double w = Math.Sqrt(1.0 - E2 * s * s);                                       // ellipsoid helper term
        double n = A / w;                                                             // prime vertical radius of curvature [m]
        double m = A * (1.0 - E2) / (w * w * w);                                      // meridian radius of curvature [m]
        lat = referenceLatitude + (enu.y / m) * 180.0 / Math.PI;                      // north offset → latitude
        lon = referenceLongitude + (enu.x / (n * Math.Cos(lat0))) * 180.0 / Math.PI;  // east offset → longitude
        alt = referenceAltitude + enu.z;                                              // up offset → altitude
    }

    // First-order Gauss-Markov: e(k+1) = φ·e(k) + √(1−φ²)·σ·w, φ = exp(−dt/τ)
    void UpdateCorrelatedError(float dt)
    {
        if (errorCorrelationTime <= 0f) { correlatedError = ScaledGaussian(CorrelatedFraction); return; } // white noise only
        float phi = Mathf.Exp(-dt / errorCorrelationTime);                            // memory of the previous error
        correlatedError = phi * correlatedError + Mathf.Sqrt(1f - phi * phi) * ScaledGaussian(CorrelatedFraction); // keeps the std constant
    }

    // Variances add up: correlated² + white² = 1, so the total std is the configured one
    float CorrelatedFraction => Mathf.Sqrt(1f - whiteNoiseFraction * whiteNoiseFraction);

    // ENU Gaussian vector with the configured horizontal/vertical std, times a fraction
    Vector3 ScaledGaussian(float fraction) => new Vector3(
        SensorUtils.Gaussian(rng) * horizontalStd,                                    // east
        SensorUtils.Gaussian(rng) * horizontalStd,                                    // north
        SensorUtils.Gaussian(rng) * verticalStd) * fraction;                          // up
}
