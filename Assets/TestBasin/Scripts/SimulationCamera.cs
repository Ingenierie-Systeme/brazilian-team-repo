using UnityEngine;
using UnityEngine.InputSystem;

// Scene camera with two modes:
//   Follow (default): chases the target from behind with smoothing.
//   Free: fly-through camera; it only moves/looks while the right mouse button is held,
//         so WASD/QE keep driving the robot otherwise.
// C switches between the two modes.
public class SimulationCamera : MonoBehaviour
{
    public enum Mode { Follow, Free }                 // available camera modes

    [Header("Mode")]
    public Mode mode = Mode.Follow;                   // current mode
    public Key toggleKey = Key.C;                     // key that switches the mode

    [Header("Follow")]
    public Transform target;                          // object to follow (the robot)
    public Vector3 targetLocalForward = Vector3.forward; // target's "front" in its local axes (the AUV uses -X)
    public float distance = 6f;                       // distance behind the target [m]
    public float height = 2f;                         // height above the target [m]
    public float smoothTime = 0.3f;                   // position smoothing [s]

    [Header("Free")]
    public float speed = 5f;                          // normal speed [m/s]
    public float fastMultiplier = 4f;                 // multiplier while holding Shift
    public float lookSensitivity = 0.15f;             // degrees per mouse pixel

    private Vector3 velocity;                         // SmoothDamp state
    private float yaw;                                // free-mode horizontal angle [degrees]
    private float pitch;                              // free-mode vertical angle [degrees]

    // Reads the initial orientation for the free mode
    void Start()
    {
        ReadAngles();
    }

    // Handles the mode switch and the free mode (input is read every frame)
    void Update()
    {
        Keyboard kb = Keyboard.current;
        if (kb != null && kb[toggleKey].wasPressedThisFrame)
        {
            mode = mode == Mode.Follow ? Mode.Free : Mode.Follow;
            ReadAngles();
        }
        if (mode == Mode.Free) UpdateFree(kb, Mouse.current);
    }

    // Follow mode runs after the physics has moved the target
    void LateUpdate()
    {
        if (mode != Mode.Follow || target == null) return;
        Vector3 back = -target.TransformDirection(targetLocalForward);
        back.y = 0f;                                                            // only the horizontal heading matters
        if (back.sqrMagnitude < 1e-4f) back = Vector3.back;                     // fallback when looking straight up/down
        back.Normalize();
        Vector3 desired = target.position + back * distance + Vector3.up * height;
        transform.position = Vector3.SmoothDamp(transform.position, desired, ref velocity, smoothTime);
        transform.rotation = Quaternion.LookRotation(target.position - transform.position);
    }

    // Free mode: look and move only while the right mouse button is held
    void UpdateFree(Keyboard kb, Mouse mouse)
    {
        if (kb == null || mouse == null || !mouse.rightButton.isPressed) return;

        Vector2 delta = mouse.delta.ReadValue();
        yaw += delta.x * lookSensitivity;
        pitch = Mathf.Clamp(pitch - delta.y * lookSensitivity, -89f, 89f);      // turn up/down without flipping
        transform.rotation = Quaternion.Euler(pitch, yaw, 0f);

        Vector3 move = Vector3.zero;
        if (kb.wKey.isPressed) move += Vector3.forward;
        if (kb.sKey.isPressed) move += Vector3.back;
        if (kb.dKey.isPressed) move += Vector3.right;
        if (kb.aKey.isPressed) move += Vector3.left;
        if (kb.eKey.isPressed) move += Vector3.up;
        if (kb.qKey.isPressed) move += Vector3.down;
        float v = speed * (kb.leftShiftKey.isPressed ? fastMultiplier : 1f);
        transform.Translate(move.normalized * v * Time.deltaTime, Space.Self);
    }

    // Stores the current orientation as free-mode angles
    void ReadAngles()
    {
        Vector3 e = transform.eulerAngles;
        yaw = e.y;
        pitch = e.x > 180f ? e.x - 360f : e.x;                                  // vertical in [-180, 180]
    }
}
