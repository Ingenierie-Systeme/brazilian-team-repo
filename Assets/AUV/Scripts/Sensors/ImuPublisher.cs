using UnityEngine;                                    // core Unity API
using Unity.Robotics.ROSTCPConnector;                 // ROSConnection
using RosMessageTypes.Sensor;                         // ImuMsg
using RosMessageTypes.Geometry;                       // QuaternionMsg, Vector3Msg, TwistStampedMsg, TwistMsg

// Inertial sensor of the AUV (accelerometer + gyroscope + orientation), published as sensor_msgs/Imu.
// Ground-truth motion comes from the Rigidbody; the measurements are what a real IMU would report:
//   - angular velocity in the sensor frame (gyroscope);
//   - specific force f = dv/dt − g in the sensor frame (accelerometer: reads +9.81 upwards at rest);
//   - orientation of the sensor (ROS frame: x = Unity +Z, y = Unity −X, z = up).
// Optional Gaussian noise and constant bias make it closer to a real sensor.
public class ImuPublisher : MonoBehaviour
{
    [Header("Sensor")]                                // Inspector section title
    public float publishRate = 50f;                   // [Hz]

    [Header("Noise and bias")]                        // Inspector section title
    public bool addNoise = true;                      // enable white noise
    public float gyroNoiseStd = 0.01f;                // gyroscope noise [rad/s]
    public float accelNoiseStd = 0.05f;               // accelerometer noise [m/s²]
    public Vector3 gyroBias = Vector3.zero;           // gyroscope bias, ROS frame [rad/s]
    public Vector3 accelBias = Vector3.zero;          // accelerometer bias, ROS frame [m/s²]

    [Header("ROS 2")]                                 // Inspector section title
    public string imuTopic = "imu/data";              // sensor_msgs/Imu topic
    public bool publishVelocity = true;               // also publish the ground-truth body velocity
    public string velocityTopic = "ins/velocity";     // geometry_msgs/TwistStamped topic
    public string frameId = "imu_link";               // frame name in the message header

    [Header("Debug (read-only, ROS frame)")]          // Inspector section title
    public Vector3 angularVelocity;                   // last gyroscope reading [rad/s]
    public Vector3 linearAcceleration;                // last accelerometer reading [m/s²]
    public int messagesPublished;                     // messages sent

    private Rigidbody rb;                             // robot Rigidbody (on this object or a parent)
    private ROSConnection ros;                        // ROS connection
    private Vector3 previousVelocity;                 // velocity at the previous physics step (world)
    private bool hasPrevious;                         // previous velocity available?
    private float timeSinceLastPublish;               // time since the last message [s]
    private readonly System.Random rng = new System.Random(); // noise generator

    // Finds the Rigidbody and registers the publishers
    void Start()
    {
        rb = GetComponentInParent<Rigidbody>();                                       // the sensor is mounted on the robot
        if (rb == null) Debug.LogError("ImuPublisher: no Rigidbody on this object or its parents", this); // misconfigured
        ros = ROSConnection.GetOrCreateInstance();                                    // shared ROS connection
        ros.RegisterPublisher<ImuMsg>(imuTopic);                                      // IMU topic
        if (publishVelocity) ros.RegisterPublisher<TwistStampedMsg>(velocityTopic);   // velocity topic
    }

    // Velocities change in the physics step, so the derivative is taken here
    void FixedUpdate()
    {
        if (rb == null) return;                                                       // nothing to measure
        float dt = Time.fixedDeltaTime;                                               // physics step [s]
        Vector3 velocity = rb.GetPointVelocity(transform.position);                   // velocity of the sensor point (includes rotation)
        if (!hasPrevious) { previousVelocity = velocity; hasPrevious = true; return; } // first step: no derivative yet
        Vector3 accelWorld = (velocity - previousVelocity) / dt;                      // a ≈ Δv / Δt
        previousVelocity = velocity;                                                  // keep for the next step

        timeSinceLastPublish += dt;                                                   // rate control
        if (timeSinceLastPublish < 1f / publishRate - 1e-4f) return;                  // not yet time (tolerance: 50 × 0.02 s is not exact in float)
        timeSinceLastPublish = 0f;                                                    // restart the counter

        Vector3 fBody = transform.InverseTransformDirection(accelWorld - Physics.gravity); // specific force in the sensor frame (Unity axes)
        Vector3 wBody = transform.InverseTransformDirection(rb.angularVelocity);      // angular velocity in the sensor frame (Unity axes)
        Vector3 accel = SensorUtils.ToRos(fBody) + accelBias;                         // to ROS FLU + bias
        Vector3 gyro = SensorUtils.AxialToRos(wBody) + gyroBias;                      // to ROS FLU + bias
        if (addNoise)                                                                 // white noise
        {
            accel += SensorUtils.GaussianVector(rng, accelNoiseStd);                  // accelerometer noise
            gyro += SensorUtils.GaussianVector(rng, gyroNoiseStd);                    // gyroscope noise
        }
        Quaternion q = SensorUtils.QuaternionToRos(transform.rotation);               // sensor orientation in ROS axes
        linearAcceleration = accel;                                                   // debug
        angularVelocity = gyro;                                                       // debug

        var header = SensorUtils.Header(frameId);                                     // shared header
        ros.Publish(imuTopic, new ImuMsg(                                             // sensor_msgs/Imu
            header,
            new QuaternionMsg(q.x, q.y, q.z, q.w),                                    // orientation
            SensorUtils.Diagonal(1e-6),                                               // orientation covariance
            new Vector3Msg(gyro.x, gyro.y, gyro.z),                                   // angular velocity
            SensorUtils.Diagonal(addNoise ? gyroNoiseStd * gyroNoiseStd : 0.0),       // its covariance
            new Vector3Msg(accel.x, accel.y, accel.z),                                // linear acceleration
            SensorUtils.Diagonal(addNoise ? accelNoiseStd * accelNoiseStd : 0.0)));   // its covariance

        if (publishVelocity)                                                          // ground-truth body velocity
        {
            Vector3 v = SensorUtils.ToRos(transform.InverseTransformDirection(velocity)); // linear [m/s]
            Vector3 w = SensorUtils.AxialToRos(wBody);                                // angular [rad/s], no noise
            ros.Publish(velocityTopic, new TwistStampedMsg(header, new TwistMsg(new Vector3Msg(v.x, v.y, v.z), new Vector3Msg(w.x, w.y, w.z))));
        }
        messagesPublished++;                                                          // count
    }
}
