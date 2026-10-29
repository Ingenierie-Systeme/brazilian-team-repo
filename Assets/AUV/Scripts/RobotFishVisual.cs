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
//
// Exaggerated display of the moving mass: when the robot is close to neutral, its center of mass is only a few
// millimeters below its center of buoyancy, so a few millimeters of moving mass already tilt it by tens of degrees
// and the real motion is hard to see. In the Exaggerated mode, the offset of the mass from its rest position is
// multiplied by a scale chosen so that the end of the travel matches `referencePitch`: the displayed mass is at
// the end of the tube when the robot, at rest, would be pitched by `referencePitch`. The scale is never below 1
// (a heavy mass far from neutral is shown as it is). The physics always uses the real position.
[ExecuteAlways]
public class RobotFishVisual : MonoBehaviour
{
    public enum MovingMassDisplay { Realistic, Exaggerated }

    [Header("Source of the actuator state")]
    public RobotFishPhysics physics;                  // empty = search on this object or its parents

    [Header("Moving parts")]
    [Tooltip("Moving mass (carriage + weight), centered on the tube axis.")]
    public Transform movingMass;
    [Tooltip("Hub + arm + fin, pivot = tail servo axis.")]
    public Transform tailPivot;
    [Tooltip("Point on the vertical tail servo axis, local axes [m].")]
    public Vector3 tailAxis;

    [Header("Moving mass display")]
    [Tooltip("Realistic: the real position. Exaggerated: offset from the rest position scaled up so that the motion is visible (visual only).")]
    public MovingMassDisplay movingMassDisplay = MovingMassDisplay.Exaggerated;
    [Tooltip("Exaggerated mode: pitch at rest [deg] that corresponds to the mass displayed at the end of its travel.")]
    [Range(10f, 89f)] public float referencePitch = 45f;

    [Header("Debug (read-only)")]
    public float displayScale = 1f;                   // displayed offset / real offset
    public float displayedPosition;                   // position of the displayed mass [m], + = forward

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

        displayScale = movingMassDisplay == MovingMassDisplay.Exaggerated ? ExaggerationScale() : 1f;
        float rest = physics != null ? physics.movingMassRestPosition : 0f;
        float travel = physics != null ? physics.movingMassTravel : 0.15f;
        displayedPosition = Mathf.Clamp(rest + (position - rest) * displayScale, -travel, travel);

        if (movingMass != null) movingMass.localPosition = Vector3.left * displayedPosition;   // forward = −X
        if (tailPivot != null)
        {
            tailPivot.localPosition = tailAxis;
            tailPivot.localRotation = Quaternion.Euler(0f, -fin, 0f);          // same convention as RobotFishPhysics
        }
    }

    // Scale between the real and the displayed offset of the moving mass.
    // Static balance about the center of buoyancy B: the robot rests when its center of mass G is right under B.
    // Moving the mass m by Δx shifts G along the tube by m·Δx/M (M = total mass); G is BG below B along the
    // vertical axis of the robot, so the robot rests at tan θ = (m·Δx / M) / BG. The offset that gives the
    // reference pitch is Δx_ref = M·BG·tan θ_ref / m, and the scale maps Δx_ref to the free travel ahead of
    // the rest position.
    float ExaggerationScale()
    {
        if (physics == null || physics.movingMass < 1e-4f) return 1f;
        float total = physics.structureMass + physics.movingMass;
        float gy = physics.structureMass * physics.structureCenterOfMass.y / total;   // the moving mass is on the axis (y = 0)
        float bg = physics.centerOfBuoyancy.y - gy;                                    // B above G [m]
        if (bg < 1e-4f) return 1f;                                                     // not computed yet (edit mode) or unstable
        float referenceOffset = total * bg * Mathf.Tan(referencePitch * Mathf.Deg2Rad) / physics.movingMass;
        float freeTravel = physics.movingMassTravel - Mathf.Abs(physics.movingMassRestPosition);
        return Mathf.Max(1f, freeTravel / referenceOffset);
    }
}
