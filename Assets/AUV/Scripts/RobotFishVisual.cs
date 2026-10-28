using UnityEngine;                                    // core Unity API

// Moves the visual parts of the detailed robot fish (RobotFishDetailed prefab) from the state of RobotFishPhysics:
//   - moving mass: a small square carriage with a metal weight on top (like on the real robot), on the tube axis,
//     placed at the current moving-mass position. Its size is fixed: it does not change with the weight set in
//     RobotFishPhysics (the weight only matters for the physics);
//   - tail: the servo hub, the arm and the fin turn together about the vertical axis of the D955TW servo.
// The CAD sliding mechanism (HS-5496MH servo, horn, rod and carriage at the front of the tube) is shown at its CAD
// pose but not moved: how the real moving mass is driven is not known (see Docs/RobotFish_Model.md).
// This component only moves visuals: the physics (mass, center of mass, fin forces) is entirely in RobotFishPhysics.
// Runs in edit mode too, so the prefab shows the parts at their reference pose (mass at rest, fin at 0°).
[ExecuteAlways]
public class RobotFishVisual : MonoBehaviour
{
    [Header("Source of the actuator state")]
    public RobotFishPhysics physics;                  // empty = search on this object or its parents

    [Header("Moving parts")]
    [Tooltip("Moving mass (carriage + weight), centered on the tube axis.")]
    public Transform movingMass;
    [Tooltip("Hub + arm + fin, pivot = tail servo axis.")]
    public Transform tailPivot;
    [Tooltip("Point on the vertical tail servo axis, local axes [m].")]
    public Vector3 tailAxis;

    void OnEnable()
    {
        if (physics == null) physics = GetComponentInParent<RobotFishPhysics>();
    }

    void LateUpdate() => Refresh();

    // Places the moving parts from the current actuator state (also callable from editor tools)
    public void Refresh()
    {
        float position = physics != null ? physics.movingMassPosition : 0f;   // + = forward (−X)
        float fin = physics != null ? physics.finAngle : 0f;                  // + = trailing edge to the right

        if (movingMass != null) movingMass.localPosition = Vector3.left * position;   // forward = −X
        if (tailPivot != null)
        {
            tailPivot.localPosition = tailAxis;
            tailPivot.localRotation = Quaternion.Euler(0f, -fin, 0f);          // same convention as RobotFishPhysics
        }
    }
}
