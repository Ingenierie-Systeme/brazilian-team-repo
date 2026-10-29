using UnityEngine;                                    // core Unity API
using Unity.Robotics.ROSTCPConnector;                 // ROSConnection
using RosMessageTypes.Std;                            // Float64Msg

// Receives the actuator commands of the robot fish from ROS 2 and hands them to RobotFishPhysics.
//
// One std_msgs/Float64 topic per actuator, every value normalized:
//   /cmd/tail_frequency  flapping frequency, fraction of maxTailFrequency (3 Hz)   [0, 1]
//   /cmd/tail_amplitude  flapping amplitude, fraction of maxTailAngle (60°)         [0, 1]
//   /cmd/tail_bias       mean fin angle, fraction of maxTailAngle (+ turn right)    [-1, 1]
//   /cmd/thrust   shortcut: swim forward, flapping at cruiseFrequency with thrust × cruiseAmplitude   [-1, 1]
//                 (negative values do nothing: a fish cannot swim backward; SimpleThrust model: forward force)
//   /cmd/rudder   shortcut: fin bias, fraction of rudderAngle (30°) (+ turn right)   [-1, 1]
//                 The shortcuts are used only while the three tail commands are 0.
//   /cmd/pitch    target position of the sliding mass    (+ nose down)              [-1, 1]
//   /cmd/ballast  ballast pump speed                      (+ fill = heavier, - empty = lighter)
//                 SIMULATION ONLY: the real robot has no ballast; the GNC never uses this topic,
//                 it only helps to set up tests (e.g. bring the AUV to a depth).
//
// While commands keep arriving, RobotFishPhysics runs in external control (keyboard ignored).
// Watchdog: if no command arrives for `commandTimeout` seconds, every actuator goes to a safe state
// (no flapping, fin centered, mass centered, pump stopped) and the keyboard takes over again.
// A real AUV does the same so that it does not keep running if its control computer stops.
[DefaultExecutionOrder(-50)]                          // runs before RobotFishPhysics (order 0) in every physics step
public class ActuatorCommandSubscriber : MonoBehaviour
{
    [Header("Target")]                                // Inspector section title
    public RobotFishPhysics robot;                // robot to command (empty = search on this object or its parents)

    [Header("ROS 2")]                                 // Inspector section title
    public string thrustTopic = "cmd/thrust";         // std_msgs/Float64, [-1, 1]
    public string rudderTopic = "cmd/rudder";         // std_msgs/Float64, [-1, 1]
    public string pitchTopic = "cmd/pitch";           // std_msgs/Float64, [-1, 1]
    public string ballastTopic = "cmd/ballast";       // std_msgs/Float64, [-1, 1], simulation only (no ballast on the real AUV)
    public string tailFrequencyTopic = "cmd/tail_frequency";   // std_msgs/Float64, [0, 1]
    public string tailAmplitudeTopic = "cmd/tail_amplitude";   // std_msgs/Float64, [0, 1]
    public string tailBiasTopic = "cmd/tail_bias";             // std_msgs/Float64, [-1, 1]

    [Header("Safety")]                                // Inspector section title
    [Tooltip("Time without any command after which the actuators go to the safe state and the keyboard takes over [s].")]
    public float commandTimeout = 0.5f;

    [Header("Debug (read-only)")]                     // Inspector section title
    public bool rosInControl;                         // commands are arriving from ROS
    public float lastThrust;                          // last value received on each topic
    public float lastRudder;
    public float lastPitch;
    public float lastBallast;
    public float lastTailFrequency;
    public float lastTailAmplitude;
    public float lastTailBias;
    public int commandsReceived;                      // messages received (all topics)

    private float lastCommandTime = float.NegativeInfinity; // Time.time of the last message [s]

    // Finds the AUV and subscribes to the topics
    void Start()
    {
        if (robot == null) robot = GetComponentInParent<RobotFishPhysics>();   // usually on the robot root
        if (robot == null) { Debug.LogError("ActuatorCommandSubscriber: no RobotFishPhysics found", this); enabled = false; return; }

        ROSConnection ros = ROSConnection.GetOrCreateInstance();                       // shared ROS connection
        ros.Subscribe<Float64Msg>(thrustTopic, msg => lastThrust = Receive(msg));       // each callback stores the
        ros.Subscribe<Float64Msg>(rudderTopic, msg => lastRudder = Receive(msg));       // clamped value and refreshes
        ros.Subscribe<Float64Msg>(pitchTopic, msg => lastPitch = Receive(msg));         // the watchdog
        ros.Subscribe<Float64Msg>(ballastTopic, msg => lastBallast = Receive(msg));
        ros.Subscribe<Float64Msg>(tailFrequencyTopic, msg => lastTailFrequency = Mathf.Max(0f, Receive(msg)));   // [0, 1]
        ros.Subscribe<Float64Msg>(tailAmplitudeTopic, msg => lastTailAmplitude = Mathf.Max(0f, Receive(msg)));   // [0, 1]
        ros.Subscribe<Float64Msg>(tailBiasTopic, msg => lastTailBias = Receive(msg));
    }

    // Common part of the callbacks (they run on the Unity main thread)
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
            robot.externalControl = true;                                          // RobotFishPhysics ignores the keyboard
            robot.thrustCommand = lastThrust;
            robot.rudderCommand = lastRudder;
            robot.pitchCommand = lastPitch;
            robot.ballastCommand = lastBallast;
            robot.tailFrequencyCommand = lastTailFrequency;
            robot.tailAmplitudeCommand = lastTailAmplitude;
            robot.tailBiasCommand = lastTailBias;
        }
        else if (rosInControl)                                                         // timeout: commands just stopped
        {
            lastThrust = lastRudder = lastPitch = lastBallast = 0f;                    // forget the old commands
            lastTailFrequency = lastTailAmplitude = lastTailBias = 0f;
            robot.thrustCommand = robot.rudderCommand = 0f;                    // safe state: no flapping, fin centered,
            robot.pitchCommand = robot.ballastCommand = 0f;                    // mass back to the center, pump stopped
            robot.tailFrequencyCommand = robot.tailAmplitudeCommand = robot.tailBiasCommand = 0f;
            robot.externalControl = false;                                         // the keyboard takes over again
            Debug.Log("ActuatorCommandSubscriber: no command for " + commandTimeout + " s, back to keyboard control", this);
        }
        rosInControl = fresh;                                                          // debug / state for the next step
    }

    // Called by the editor when the component is added: fills the target automatically
    void Reset()
    {
        robot = GetComponentInParent<RobotFishPhysics>();                          // fill the field when added in the editor
    }
}
