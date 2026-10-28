using UnityEngine;
using UnityEngine.InputSystem;

public class SubmarinePhysics : MonoBehaviour
{
    [Header("Orientation")]
    public bool useXAxisAsForward = true;
    public bool invertForward = true;

    [Header("Environment")]
    public float waterLevel = 0f;
    [Tooltip("Depth over which a hull vertex goes from no buoyancy to full buoyancy (about the hull diameter).")]
    public float submersionRange = 0.2f;

    [Header("Propulsion & Steering")]
    public float thrustForce = 30f;
    public float rudderTorque = 5f;

    [Header("Pitch Control (Mass Shifting)")]
    public float maxPitchShift = 0.3f;
    public float pitchShiftSpeed = 1f;

    [Header("Ballast Control")]
    public float ballastRate = 0.25f;

    private Rigidbody rb;
    private MeshFilter submarineMesh;
    private Transform meshTransform;
    private Transform finPoint;
    private Vector3[] localVertices;
    private Vector3 baseCenterOfMass;
    private float currentShift = 0f;
    private float ballastLevel = 0.5f;

    private const float WaterDensity = 1000f;
    private const float WaterLinearDamping = 2.5f;
    private const float WaterAngularDamping = 2.5f;
    private const float AirLinearDamping = 0.05f;
    private const float AirAngularDamping = 0.05f;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        if (rb == null) return;

        baseCenterOfMass = new Vector3(rb.centerOfMass.x, -0.05f, rb.centerOfMass.z);
        rb.centerOfMass = baseCenterOfMass;

        submarineMesh = GetComponentInChildren<MeshFilter>();
        if (submarineMesh != null && submarineMesh.sharedMesh != null)
        {
            meshTransform = submarineMesh.transform;
            localVertices = submarineMesh.sharedMesh.vertices;
        }

        Transform foundFin = transform.Find("FinActuationPoint");
        finPoint = foundFin != null ? foundFin : transform;
    }

    void FixedUpdate()
    {
        if (rb == null) return;

        UpdateBallast();
        ApplyBuoyancyAndDamping();
        UpdatePitchMassShift();
        ApplyPropulsionAndSteering();
    }

    private Vector3 GetLocalForward()
    {
        Vector3 forward = useXAxisAsForward ? Vector3.right : Vector3.forward;
        return invertForward ? -forward : forward;
    }

    private Vector3 GetWorldForward()
    {
        return transform.TransformDirection(GetLocalForward());
    }

    private void UpdateBallast()
    {
        if (Keyboard.current == null) return;

        float input = 0f;
        if (Keyboard.current.qKey.isPressed || Keyboard.current.leftShiftKey.isPressed) input += 1f;
        if (Keyboard.current.eKey.isPressed || Keyboard.current.leftCtrlKey.isPressed) input -= 1f;

        ballastLevel = Mathf.Clamp01(ballastLevel + input * ballastRate * Time.fixedDeltaTime);
    }

    private void ApplyBuoyancyAndDamping()
    {
        if (localVertices == null || localVertices.Length == 0 || meshTransform == null) return;

        float gravity = Mathf.Abs(Physics.gravity.y);
        float neutralVolume = rb.mass / WaterDensity;
        float effectiveVolume = neutralVolume * Mathf.Lerp(1.25f, 0.75f, ballastLevel);
        float volumePerVertex = effectiveVolume / localVertices.Length;

        int submergedCount = 0;

        foreach (Vector3 localVertex in localVertices)
        {
            Vector3 worldPos = meshTransform.TransformPoint(localVertex);
            float depth = waterLevel - worldPos.y;

            if (depth > 0f)
            {
                submergedCount++;
                float submersion = Mathf.Clamp01(depth / Mathf.Max(submersionRange, 0.001f));
                float force = WaterDensity * gravity * volumePerVertex * submersion;
                rb.AddForceAtPosition(Vector3.up * force, worldPos, ForceMode.Force);
            }
        }

        float ratio = (float)submergedCount / localVertices.Length;
        rb.linearDamping = Mathf.Lerp(AirLinearDamping, WaterLinearDamping, ratio);
        rb.angularDamping = Mathf.Lerp(AirAngularDamping, WaterAngularDamping, ratio);
    }

    private void UpdatePitchMassShift()
    {
        float input = 0f;
        if (Keyboard.current != null)
        {
            if (Keyboard.current.wKey.isPressed || Keyboard.current.upArrowKey.isPressed) input = 1f;
            if (Keyboard.current.sKey.isPressed || Keyboard.current.downArrowKey.isPressed) input = -1f;
        }

        float target = input * maxPitchShift;
        currentShift = Mathf.MoveTowards(currentShift, target, pitchShiftSpeed * Time.fixedDeltaTime);
        rb.centerOfMass = baseCenterOfMass + GetLocalForward() * currentShift;
    }

    private void ApplyPropulsionAndSteering()
    {
        float thrust = 0f;
        float steer = 0f;

        if (Keyboard.current != null)
        {
            if (Keyboard.current.spaceKey.isPressed) thrust = 1f;
            if (Keyboard.current.dKey.isPressed || Keyboard.current.rightArrowKey.isPressed) steer = 1f;
            if (Keyboard.current.aKey.isPressed || Keyboard.current.leftArrowKey.isPressed) steer = -1f;
        }

        if (thrust > 0f)
        {
            Vector3 force = GetWorldForward() * (thrustForce * thrust);
            rb.AddForceAtPosition(force, finPoint.position, ForceMode.Force);
        }

        if (Mathf.Abs(steer) > 0.01f)
        {
            rb.AddTorque(Vector3.up * (steer * rudderTorque), ForceMode.Force);
        }
    }

    void OnDrawGizmosSelected()
    {
        Gizmos.color = new Color(0f, 0.6f, 1f, 0.35f);
        Gizmos.DrawCube(new Vector3(transform.position.x, waterLevel, transform.position.z), new Vector3(10f, 0.02f, 10f));

        if (rb != null)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawSphere(transform.TransformPoint(rb.centerOfMass), 0.08f);
        }

        Gizmos.color = Color.red;
        Vector3 origin = finPoint != null ? finPoint.position : transform.position;
        Gizmos.DrawRay(origin, GetWorldForward() * 2f);
    }
}
