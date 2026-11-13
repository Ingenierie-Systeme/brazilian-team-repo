using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;

// Inertial sensor of the AUV (accelerometer + gyroscope + orientation), published as sensor_msgs/Imu.
// Ground-truth motion comes from the Rigidbody; the measurements are what a real IMU would report:
//   - angular velocity in the sensor frame (gyroscope);
//   - specific force f = dv/dt − g in the sensor frame (accelerometer: reads +9.81 upwards at rest);
//   - orientation of the sensor (ROS frame: x = Unity +Z, y = Unity −X, z = up).
// Optional Gaussian noise and constant bias make it closer to a real sensor.
public class ImuPublisher : MonoBehaviour
{
    [Header("Sensor")]
    public float publishRate = 50f;                   // [Hz]

    [Header("Noise and bias")]
    public bool addNoise = true;                      // enable white noise
    public float gyroNoiseStd = 0.01f;                // gyroscope noise [rad/s]
    public float accelNoiseStd = 0.05f;               // accelerometer noise [m/s²]
    public Vector3 gyroBias = Vector3.zero;           // gyroscope bias, ROS frame [rad/s]
    public Vector3 accelBias = Vector3.zero;          // accelerometer bias, ROS frame [m/s²]

    [Header("ROS 2")]
    public string imuTopic = "imu/data";              // sensor_msgs/Imu topic
    public bool publishVelocity = true;               // also publish the ground-truth body velocity
    public string velocityTopic = "ins/velocity";     // geometry_msgs/TwistStamped topic
    public string frameId = "imu_link";               // frame name in the message header

    [Header("Debug (read-only, ROS frame)")]
    public Vector3 angularVelocity;                   // last gyroscope reading [rad/s]
    public Vector3 linearAcceleration;                // last accelerometer reading [m/s²]
    public int messagesPublished;                     // messages sent

    private Rigidbody rb;                             // robot Rigidbody (on this object or a parent)
    private ROSConnection ros;
    private Vector3 previousVelocity;                 // velocity at the previous physics step (world)
    private bool hasPrevious;                         // previous velocity available?
    private float timeSinceLastPublish;               // time since the last message [s]
    private readonly System.Random rng = new System.Random(); // noise generator

    // Finds the Rigidbody and registers the publishers
    void Start()
    {
        rb = GetComponentInParent<Rigidbody>();
        if (rb == null) Debug.LogError("ImuPublisher: no Rigidbody on this object or its parents", this);
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImuMsg>(imuTopic);
        if (publishVelocity) ros.RegisterPublisher<TwistStampedMsg>(velocityTopic);
    }

    // Velocities change in the physics step, so the derivative is taken here
    void FixedUpdate()
    {
        if (rb == null) return;
        float dt = Time.fixedDeltaTime;
        Vector3 velocity = rb.GetPointVelocity(transform.position);                   // velocity of the sensor point (includes rotation)
        if (!hasPrevious) { previousVelocity = velocity; hasPrevious = true; return; }
        Vector3 accelWorld = (velocity - previousVelocity) / dt;                      // a ≈ Δv / Δt
        previousVelocity = velocity;

        timeSinceLastPublish += dt;
        if (timeSinceLastPublish < 1f / publishRate - 1e-4f) return;                  // not yet time (tolerance: 50 × 0.02 s is not exact in float)
        timeSinceLastPublish = 0f;

        Vector3 fBody = transform.InverseTransformDirection(accelWorld - Physics.gravity); // specific force in the sensor frame (Unity axes)
        Vector3 wBody = transform.InverseTransformDirection(rb.angularVelocity);      // angular velocity in the sensor frame (Unity axes)
        Vector3 accel = SensorUtils.ToRos(fBody) + accelBias;                         // to ROS FLU + bias
        Vector3 gyro = SensorUtils.AxialToRos(wBody) + gyroBias;                      // to ROS FLU + bias
        if (addNoise)
        {
            accel += SensorUtils.GaussianVector(rng, accelNoiseStd);
            gyro += SensorUtils.GaussianVector(rng, gyroNoiseStd);
        }
        Quaternion q = SensorUtils.QuaternionToRos(transform.rotation);
        linearAcceleration = accel;
        angularVelocity = gyro;

        var header = SensorUtils.Header(frameId);
        ros.Publish(imuTopic, new ImuMsg(
            header,
            new QuaternionMsg(q.x, q.y, q.z, q.w),
            SensorUtils.Diagonal(1e-6),
            new Vector3Msg(gyro.x, gyro.y, gyro.z),
            SensorUtils.Diagonal(addNoise ? gyroNoiseStd * gyroNoiseStd : 0.0),
            new Vector3Msg(accel.x, accel.y, accel.z),
            SensorUtils.Diagonal(addNoise ? accelNoiseStd * accelNoiseStd : 0.0)));

        if (publishVelocity)
        {
            Vector3 v = SensorUtils.ToRos(transform.InverseTransformDirection(velocity)); // linear [m/s]
            Vector3 w = SensorUtils.AxialToRos(wBody);                                // angular [rad/s], no noise
            ros.Publish(velocityTopic, new TwistStampedMsg(header, new TwistMsg(new Vector3Msg(v.x, v.y, v.z), new Vector3Msg(w.x, w.y, w.z))));
        }
        messagesPublished++;
    }
}
