using UnityEngine;                                    // core Unity API
using UnityEngine.InputSystem;                        // new Input System

// Scene camera with two modes:
//   Follow (default): chases the target from behind with smoothing.
//   Free: fly-through camera; it only moves/looks while the right mouse button is held,
//         so WASD/QE keep driving the robot otherwise.
// C switches between the two modes.
public class SimulationCamera : MonoBehaviour
{
    public enum Mode { Follow, Free }                 // available camera modes

    [Header("Mode")]                                  // Inspector section title
    public Mode mode = Mode.Follow;                   // current mode
    public Key toggleKey = Key.C;                     // key that switches the mode

    [Header("Follow")]                                // Inspector section title
    public Transform target;                          // object to follow (the robot)
    public Vector3 targetLocalForward = Vector3.forward; // target's "front" in its local axes (the AUV uses -X)
    public float distance = 6f;                       // distance behind the target [m]
    public float height = 2f;                         // height above the target [m]
    public float smoothTime = 0.3f;                   // position smoothing [s]

    [Header("Free")]                                  // Inspector section title
    public float speed = 5f;                          // normal speed [m/s]
    public float fastMultiplier = 4f;                 // multiplier while holding Shift
    public float lookSensitivity = 0.15f;             // degrees per mouse pixel

    private Vector3 velocity;                         // SmoothDamp state
    private float yaw;                                // free-mode horizontal angle [degrees]
    private float pitch;                              // free-mode vertical angle [degrees]

    // Reads the initial orientation for the free mode
    void Start()
    {
        ReadAngles();                                 // start from the current orientation
    }

    // Handles the mode switch and the free mode (input is read every frame)
    void Update()
    {
        Keyboard kb = Keyboard.current;               // current keyboard (may be null)
        if (kb != null && kb[toggleKey].wasPressedThisFrame)                     // mode key pressed?
        {
            mode = mode == Mode.Follow ? Mode.Free : Mode.Follow;               // switch the mode
            ReadAngles();                                                       // free mode starts from the current view
        }
        if (mode == Mode.Free) UpdateFree(kb, Mouse.current);                   // free mode
    }

    // Follow mode runs after the physics has moved the target
    void LateUpdate()
    {
        if (mode != Mode.Follow || target == null) return;                      // nothing to follow
        Vector3 back = -target.TransformDirection(targetLocalForward);          // direction behind the target
        back.y = 0f;                                                            // only the horizontal heading matters
        if (back.sqrMagnitude < 1e-4f) back = Vector3.back;                     // fallback when looking straight up/down
        back.Normalize();                                                       // unit vector
        Vector3 desired = target.position + back * distance + Vector3.up * height; // wanted camera position
        transform.position = Vector3.SmoothDamp(transform.position, desired, ref velocity, smoothTime); // smooth move
        transform.rotation = Quaternion.LookRotation(target.position - transform.position); // look at the target
    }

    // Free mode: look and move only while the right mouse button is held
    void UpdateFree(Keyboard kb, Mouse mouse)
    {
        if (kb == null || mouse == null || !mouse.rightButton.isPressed) return; // inactive without the right button

        Vector2 delta = mouse.delta.ReadValue();                                // mouse movement [pixels]
        yaw += delta.x * lookSensitivity;                                       // turn sideways
        pitch = Mathf.Clamp(pitch - delta.y * lookSensitivity, -89f, 89f);      // turn up/down without flipping
        transform.rotation = Quaternion.Euler(pitch, yaw, 0f);                  // apply the rotation

        Vector3 move = Vector3.zero;                                            // local movement direction
        if (kb.wKey.isPressed) move += Vector3.forward;                         // forward
        if (kb.sKey.isPressed) move += Vector3.back;                            // backward
        if (kb.dKey.isPressed) move += Vector3.right;                           // right
        if (kb.aKey.isPressed) move += Vector3.left;                            // left
        if (kb.eKey.isPressed) move += Vector3.up;                              // up
        if (kb.qKey.isPressed) move += Vector3.down;                            // down
        float v = speed * (kb.leftShiftKey.isPressed ? fastMultiplier : 1f);    // current speed
        transform.Translate(move.normalized * v * Time.deltaTime, Space.Self);  // move in the camera frame
    }

    // Stores the current orientation as free-mode angles
    void ReadAngles()
    {
        Vector3 e = transform.eulerAngles;                                      // current angles
        yaw = e.y;                                                              // horizontal
        pitch = e.x > 180f ? e.x - 360f : e.x;                                  // vertical in [-180, 180]
    }
}
