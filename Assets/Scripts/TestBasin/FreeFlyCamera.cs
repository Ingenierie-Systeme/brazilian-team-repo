using UnityEngine;                                    // core Unity API
using UnityEngine.InputSystem;                        // new Input System (the project does not use the legacy one)

// Free camera to inspect the basin in Play mode (including under water).
// Right mouse button + mouse: look | WASD: move | Q/E: down/up | Shift: fast.
public class FreeFlyCamera : MonoBehaviour
{
    public float speed = 5f;                          // normal speed [m/s]
    public float fastMultiplier = 4f;                 // multiplier while holding Shift
    public float lookSensitivity = 0.15f;             // degrees per mouse pixel

    private float yaw;                                // accumulated horizontal rotation [degrees]
    private float pitch;                              // accumulated vertical rotation [degrees]

    // Reads the initial camera orientation
    void Start()
    {
        Vector3 e = transform.eulerAngles;            // current angles
        yaw = e.y;                                    // horizontal
        pitch = e.x > 180f ? e.x - 360f : e.x;        // vertical in the [-180, 180] range
    }

    // Moves and rotates the camera every frame
    void Update()
    {
        Keyboard kb = Keyboard.current;               // current keyboard (may be null)
        Mouse mouse = Mouse.current;                  // current mouse (may be null)
        if (kb == null || mouse == null) return;      // no devices: nothing to do

        if (mouse.rightButton.isPressed)              // only look around while the right button is held
        {
            Vector2 delta = mouse.delta.ReadValue();  // mouse movement this frame [pixels]
            yaw += delta.x * lookSensitivity;         // turn sideways
            pitch = Mathf.Clamp(pitch - delta.y * lookSensitivity, -89f, 89f); // turn up/down without flipping
            transform.rotation = Quaternion.Euler(pitch, yaw, 0f); // apply the rotation
        }

        Vector3 move = Vector3.zero;                  // movement direction (local)
        if (kb.wKey.isPressed) move += Vector3.forward;   // forward
        if (kb.sKey.isPressed) move += Vector3.back;      // backward
        if (kb.dKey.isPressed) move += Vector3.right;     // right
        if (kb.aKey.isPressed) move += Vector3.left;      // left
        if (kb.eKey.isPressed) move += Vector3.up;        // up
        if (kb.qKey.isPressed) move += Vector3.down;      // down
        float v = speed * (kb.leftShiftKey.isPressed ? fastMultiplier : 1f); // current speed
        transform.Translate(move.normalized * v * Time.deltaTime, Space.Self); // move in the camera frame
    }
}
