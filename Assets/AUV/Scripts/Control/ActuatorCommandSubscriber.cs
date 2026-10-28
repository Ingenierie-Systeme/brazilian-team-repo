using UnityEngine;                                    // core Unity API
using Unity.Robotics.ROSTCPConnector;                 // ROSConnection
using RosMessageTypes.Std;                            // Float64Msg

// Receives the actuator commands of the AUV from ROS 2 and hands them to SubmarinePhysics.
//
// One std_msgs/Float64 topic per actuator, every value normalized to [-1, 1]:
//   /cmd/thrust   fraction of the maximum thrust          (+ forward, - reverse)
//   /cmd/rudder   fraction of the maximum rudder torque   (+ turn right)
//   /cmd/pitch    target position of the sliding mass    (+ nose down)
//   /cmd/ballast  ballast pump speed                      (+ fill = heavier, - empty = lighter)
//
// While commands keep arriving, SubmarinePhysics runs in external control (keyboard ignored).
// Watchdog: if no command arrives for `commandTimeout` seconds, every actuator goes to a safe state
// (no thrust, rudder centered, mass centered, pump stopped) and the keyboard takes over again.
// A real AUV does the same so that it does not keep running if its control computer stops.
[DefaultExecutionOrder(-50)]                          // runs before SubmarinePhysics (order 0) in every physics step
public class ActuatorCommandSubscriber : MonoBehaviour
{
    [Header("Target")]                                // Inspector section title
    public SubmarinePhysics submarine;                // AUV to command (empty = search on this object or its parents)

    [Header("ROS 2")]                                 // Inspector section title
    public string thrustTopic = "cmd/thrust";         // std_msgs/Float64, [-1, 1]
    public string rudderTopic = "cmd/rudder";         // std_msgs/Float64, [-1, 1]
    public string pitchTopic = "cmd/pitch";           // std_msgs/Float64, [-1, 1]
    public string ballastTopic = "cmd/ballast";       // std_msgs/Float64, [-1, 1]

    [Header("Safety")]                                // Inspector section title
    [Tooltip("Time without any command after which the actuators go to the safe state and the keyboard takes over [s].")]
    public float commandTimeout = 0.5f;

    [Header("Debug (read-only)")]                     // Inspector section title
    public bool rosInControl;                         // commands are arriving from ROS
    public float lastThrust;                          // last value received on each topic
    public float lastRudder;
    public float lastPitch;
    public float lastBallast;
    public int commandsReceived;                      // messages received (all topics)

    private float lastCommandTime = float.NegativeInfinity; // Time.time of the last message [s]

    // Finds the AUV and subscribes to the four topics
    void Start()
    {
        if (submarine == null) submarine = GetComponentInParent<SubmarinePhysics>();   // usually on the AUV root
        if (submarine == null) { Debug.LogError("ActuatorCommandSubscriber: no SubmarinePhysics found", this); enabled = false; return; }

        ROSConnection ros = ROSConnection.GetOrCreateInstance();                       // shared ROS connection
        ros.Subscribe<Float64Msg>(thrustTopic, msg => lastThrust = Receive(msg));       // each callback stores the
        ros.Subscribe<Float64Msg>(rudderTopic, msg => lastRudder = Receive(msg));       // clamped value and refreshes
        ros.Subscribe<Float64Msg>(pitchTopic, msg => lastPitch = Receive(msg));         // the watchdog
        ros.Subscribe<Float64Msg>(ballastTopic, msg => lastBallast = Receive(msg));
    }

    // Common part of the four callbacks (they run on the Unity main thread)
    float Receive(Float64Msg msg)
    {
        lastCommandTime = Time.time;                                                   // feeds the watchdog
        commandsReceived++;                                                            // count
        float value = (float)msg.data;                                                 // double → float
        return float.IsNaN(value) ? 0f : Mathf.Clamp(value, -1f, 1f);                  // reject NaN, keep in [-1, 1]
    }

    // Before each physics step: hand the commands over, or fall back to the safe state
    void FixedUpdate()
    {
        bool fresh = Time.time - lastCommandTime <= commandTimeout;                    // has a command arrived recently?

        if (fresh)                                                                     // ROS in control
        {
            submarine.externalControl = true;                                          // SubmarinePhysics ignores the keyboard
            submarine.thrustCommand = lastThrust;
            submarine.rudderCommand = lastRudder;
            submarine.pitchCommand = lastPitch;
            submarine.ballastCommand = lastBallast;
        }
        else if (rosInControl)                                                         // timeout: commands just stopped
        {
            lastThrust = lastRudder = lastPitch = lastBallast = 0f;                    // forget the old commands
            submarine.thrustCommand = submarine.rudderCommand = 0f;                    // safe state: no thrust, rudder centered,
            submarine.pitchCommand = submarine.ballastCommand = 0f;                    // mass back to the center, pump stopped
            submarine.externalControl = false;                                         // the keyboard takes over again
            Debug.Log("ActuatorCommandSubscriber: no command for " + commandTimeout + " s, back to keyboard control", this);
        }
        rosInControl = fresh;                                                          // debug / state for the next step
    }

    // Called by the editor when the component is added: fills the target automatically
    void Reset()
    {
        submarine = GetComponentInParent<SubmarinePhysics>();                          // fill the field when added in the editor
    }
}
