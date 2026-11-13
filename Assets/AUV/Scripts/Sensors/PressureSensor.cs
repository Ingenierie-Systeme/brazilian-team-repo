using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

// Pressure (depth) sensor of the AUV, like the Blue Robotics Bar30.
// Absolute pressure p = p_atm + ρ·g·depth (depth of the sensor below the basin water surface, 0 out of the water),
// published as sensor_msgs/FluidPressure [Pa]; the depth computed from the noisy pressure is also published [m].
public class PressureSensor : MonoBehaviour
{
    [Header("Environment")]
    public TestBasin basin;                           // basin providing the water surface (empty = search the scene)
    public float atmosphericPressure = 101325f;       // [Pa]
    public float waterDensity = 1000f;                // fresh water [kg/m³]

    [Header("Sensor")]
    public float publishRate = 20f;                   // [Hz]
    public bool addNoise = true;                      // enable white noise
    public float noiseStdPa = 20f;                    // noise [Pa] (Bar30 resolution ≈ 0.2 mbar = 20 Pa ≈ 2 mm of water)
    public float biasPa = 0f;                         // constant offset [Pa]

    [Header("ROS 2")]
    public string pressureTopic = "pressure";         // sensor_msgs/FluidPressure topic
    public bool publishDepth = true;                  // also publish the depth
    public string depthTopic = "depth";               // std_msgs/Float64 topic [m, positive down]
    public string frameId = "pressure_link";          // frame name in the message header

    [Header("Debug (read-only)")]
    public float trueDepth;                           // real depth of the sensor [m]
    public float measuredPressure;                    // last pressure published [Pa]
    public float measuredDepth;                       // last depth published [m]
    public int messagesPublished;                     // messages sent

    private ROSConnection ros;
    private float timeSinceLastPublish;               // time since the last message [s]
    private readonly System.Random rng = new System.Random(); // noise generator

    // Finds the basin and registers the publishers
    void Start()
    {
        if (basin == null) basin = FindAnyObjectByType<TestBasin>();
        if (basin == null) Debug.LogWarning("PressureSensor: no TestBasin in the scene; depth will be 0", this);
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<FluidPressureMsg>(pressureTopic);
        if (publishDepth) ros.RegisterPublisher<Float64Msg>(depthTopic);
    }

    // Publishes at the configured rate
    void Update()
    {
        timeSinceLastPublish += Time.deltaTime;
        if (timeSinceLastPublish < 1f / publishRate) return;
        timeSinceLastPublish = 0f;

        float g = -Physics.gravity.y;                                                 // gravity [m/s²]
        trueDepth = basin != null ? Mathf.Max(0f, basin.WaterSurfaceY - transform.position.y) : 0f; // below the surface, 0 above it
        float p = atmosphericPressure + waterDensity * g * trueDepth + biasPa;        // hydrostatic pressure + bias
        if (addNoise) p += SensorUtils.Gaussian(rng) * noiseStdPa;
        measuredPressure = p;
        measuredDepth = (p - atmosphericPressure) / (waterDensity * g);               // depth as a driver would compute it

        var header = SensorUtils.Header(frameId);
        double variance = addNoise ? (double)noiseStdPa * noiseStdPa : 0.0;           // pressure variance [Pa²]
        ros.Publish(pressureTopic, new FluidPressureMsg(header, p, variance));
        if (publishDepth) ros.Publish(depthTopic, new Float64Msg(measuredDepth));
        messagesPublished++;
    }
}
