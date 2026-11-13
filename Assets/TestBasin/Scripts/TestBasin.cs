using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

// Procedurally generates the test basin: floor, 4 walls, water volume and water surface.
// Convention: 1 unit = 1 m, Y up; cavity centered on the origin in X/Z, top of the floor at y = 0.
[ExecuteAlways]                                       // also runs in the editor, outside Play mode
[DisallowMultipleComponent]
public class TestBasin : MonoBehaviour
{
    [Header("Inner cavity [m]")]
    public float length = 20f;                        // inner length (Z axis)
    public float width = 12f;                         // inner width (X axis)
    public float depth = 6f;                          // inner depth (Y axis)
    public float wallThickness = 0.5f;                // floor and wall thickness
    public float waterLevel = 5.5f;                   // water height measured from the floor

    [Header("Materials")]
    public Material basinMaterial;                    // floor and wall material (grid or plain)
    public Material waterSurfaceMaterial;
    public Material waterVolumeMaterial;              // optional blue volume material
    public bool showWaterVolume = false;              // shows/hides the transparent blue volume

    // Fixed names of the generated children (used to reuse objects instead of recreating them)
    const string FloorName = "Floor";
    const string WallLeftName = "Wall_Left";          // wall at -X
    const string WallRightName = "Wall_Right";        // wall at +X
    const string WallFrontName = "Wall_Front";        // wall at +Z
    const string WallBackName = "Wall_Back";          // wall at -Z
    const string WaterVolumeName = "WaterVolume";     // physical volume (trigger)
    const string WaterSurfaceName = "WaterSurface";   // surface plane
    const string WaterTintName = "WaterTint";         // optional visual volume

    // Global properties read by the grid shader
    static readonly int GridMatrixId = Shader.PropertyToID("_BasinWorldToGrid");   // world→grid matrix ID
    static readonly int GridValidId = Shader.PropertyToID("_BasinGridValid");      // "matrix is valid" flag ID

    [SerializeField, HideInInspector] private BoxCollider waterCollider;
    private Matrix4x4 lastGridMatrix;                                               // last matrix sent to the shader

    // Water volume trigger collider, used by SimpleBuoyancy
    public BoxCollider WaterCollider => waterCollider;

    // World Y of the water surface
    public float WaterSurfaceY => transform.TransformPoint(0f, waterLevel, 0f).y;   // converts the local height to world space

    // Makes sure the basin exists and sends the grid matrix when the component is enabled
    void OnEnable()
    {
        if (transform.Find(FloorName) == null) Generate();
        else ApplyMaterials();
        UpdateGridMatrix(true);
    }

    // Keeps the grid aligned when the basin is moved (in the editor and in Play mode)
    void LateUpdate()
    {
        UpdateGridMatrix(false);
    }

#if UNITY_EDITOR
    // Regenerates the basin when a parameter changes in the Inspector
    void OnValidate()
    {
        EditorApplication.delayCall -= DelayedGenerate;
        EditorApplication.delayCall += DelayedGenerate;     // deferred: Unity does not allow creating objects inside OnValidate
    }

    // Runs right after OnValidate, when the hierarchy can be modified
    void DelayedGenerate()
    {
        if (this == null) return;                                    // the component may have been destroyed meanwhile
        if (PrefabUtility.IsPartOfPrefabAsset(this)) return;         // never touch the prefab asset directly
        Generate();
    }
#endif

    // Creates or updates every piece of the basin from the parameters
    [ContextMenu("Generate basin")]
    public void Generate()
    {
        float t = Mathf.Max(0.01f, wallThickness);                   // minimum thickness to avoid degenerate boxes
        float d = Mathf.Max(0.1f, depth);
        float outerX = width + 2f * t;
        float outerZ = length + 2f * t;
        float level = Mathf.Clamp(waterLevel, 0.01f, d);

        // Floor: top at y = 0, covers the whole outer area
        Piece(FloorName, new Vector3(0f, -t / 2f, 0f), new Vector3(outerX, t, outerZ));
        // Side walls (±X): span the whole outer length, covering the corners
        Piece(WallLeftName, new Vector3(-(width + t) / 2f, d / 2f, 0f), new Vector3(t, d, outerZ));
        Piece(WallRightName, new Vector3((width + t) / 2f, d / 2f, 0f), new Vector3(t, d, outerZ));
        // Front (+Z) and back (-Z) walls: fit between the side walls
        Piece(WallFrontName, new Vector3(0f, d / 2f, (length + t) / 2f), new Vector3(width, d, t));
        Piece(WallBackName, new Vector3(0f, d / 2f, -(length + t) / 2f), new Vector3(width, d, t));

        // Physical water volume: trigger from the floor (y = 0) up to the water level
        GameObject volume = Child(WaterVolumeName, null);
        volume.transform.localPosition = new Vector3(0f, level / 2f, 0f);
        volume.transform.localRotation = Quaternion.identity;
        volume.transform.localScale = Vector3.one;                           // scale 1: the size lives in the collider
        volume.layer = 2;                                                    // "Ignore Raycast" layer: does not block sensor rays
        waterCollider = volume.GetComponent<BoxCollider>();
        if (waterCollider == null) waterCollider = volume.AddComponent<BoxCollider>();
        waterCollider.isTrigger = true;                                      // trigger: detects entry, no collision
        waterCollider.center = Vector3.zero;
        waterCollider.size = new Vector3(width, level, length);

        // Water surface: Quad lying flat (facing up) at the water level
        GameObject surface = Child(WaterSurfaceName, PrimitiveType.Quad);
        RemoveCollider(surface);
        surface.transform.localPosition = new Vector3(0f, level, 0f);
        surface.transform.localRotation = Quaternion.Euler(90f, 0f, 0f);
        surface.transform.localScale = new Vector3(width, length, 1f);

        // Optional visual volume: very transparent blue box filling the water column
        GameObject tint = Child(WaterTintName, PrimitiveType.Cube);
        RemoveCollider(tint);
        tint.transform.localPosition = new Vector3(0f, level / 2f, 0f);
        tint.transform.localRotation = Quaternion.identity;
        tint.transform.localScale = new Vector3(width - 0.01f, level - 0.01f, length - 0.01f); // slightly smaller than the cavity (avoids z-fighting)
        tint.SetActive(showWaterVolume);

        ApplyMaterials();
        UpdateGridMatrix(true);
    }

    // Applies the configured materials to the generated pieces
    public void ApplyMaterials()
    {
        foreach (string n in new[] { FloorName, WallLeftName, WallRightName, WallFrontName, WallBackName })
            SetMaterial(n, basinMaterial);
        SetMaterial(WaterSurfaceName, waterSurfaceMaterial);
        SetMaterial(WaterTintName, waterVolumeMaterial);
        Transform tint = transform.Find(WaterTintName);
        if (tint != null) tint.gameObject.SetActive(showWaterVolume);
    }

    // Creates/updates a solid piece (static cube with a BoxCollider)
    void Piece(string pieceName, Vector3 center, Vector3 size)
    {
        GameObject go = Child(pieceName, PrimitiveType.Cube);
        go.transform.localPosition = center;
        go.transform.localRotation = Quaternion.identity;
        go.transform.localScale = size;
        if (go.GetComponent<BoxCollider>() == null) go.AddComponent<BoxCollider>();
#if UNITY_EDITOR                                                              // static flags only exist in the editor
        GameObjectUtility.SetStaticEditorFlags(go, StaticEditorFlags.BatchingStatic
                                                   | StaticEditorFlags.OccluderStatic | StaticEditorFlags.OccludeeStatic
                                                   | StaticEditorFlags.ReflectionProbeStatic);
#endif
    }

    // Finds a child by name; creates it (primitive or empty) if it does not exist
    GameObject Child(string childName, PrimitiveType? primitive)
    {
        Transform existing = transform.Find(childName);
        if (existing != null) return existing.gameObject;
        GameObject go = primitive.HasValue ? GameObject.CreatePrimitive(primitive.Value) : new GameObject();
        go.name = childName;
        go.transform.SetParent(transform, false);
        return go;
    }

    // Removes the collider of a visual-only object
    static void RemoveCollider(GameObject go)
    {
        Collider c = go.GetComponent<Collider>();
        if (c == null) return;
        if (Application.isPlaying) Destroy(c);
        else DestroyImmediate(c);
    }

    // Applies a material to a child's Renderer, if both exist
    void SetMaterial(string childName, Material mat)
    {
        Transform child = transform.Find(childName);
        if (child == null || mat == null) return;
        Renderer r = child.GetComponent<Renderer>();
        if (r != null && r.sharedMaterial != mat) r.sharedMaterial = mat;
    }

    // Sends the shader the matrix from world space to grid space (basin local space + alignment)
    void UpdateGridMatrix(bool force)
    {
        // Offset that puts the inner edges on integer lines when width/length are odd
        Vector3 align = new Vector3(Mathf.Repeat(-width / 2f, 1f), 0f, Mathf.Repeat(-length / 2f, 1f));
        Matrix4x4 m = Matrix4x4.Translate(align) * transform.worldToLocalMatrix; // world → basin local → aligned
        if (!force && m == lastGridMatrix) return;
        lastGridMatrix = m;
        Shader.SetGlobalMatrix(GridMatrixId, m);
        Shader.SetGlobalFloat(GridValidId, 1f);
    }

    // Draws the cavity and water outlines in the Scene view
    void OnDrawGizmosSelected()
    {
        Gizmos.matrix = transform.localToWorldMatrix;
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireCube(new Vector3(0f, depth / 2f, 0f), new Vector3(width, depth, length));
        Gizmos.color = Color.cyan;
        Gizmos.DrawWireCube(new Vector3(0f, waterLevel / 2f, 0f), new Vector3(width, waterLevel, length));
    }
}
