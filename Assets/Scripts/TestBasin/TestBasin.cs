using UnityEngine;                                    // core Unity API
#if UNITY_EDITOR                                      // editor-only block
using UnityEditor;                                    // editor API (static flags, delayCall, prefabs)
#endif                                                // end of editor-only block

// Procedurally generates the test basin: floor, 4 walls, water volume and water surface.
// Convention: 1 unit = 1 m, Y up; cavity centered on the origin in X/Z, top of the floor at y = 0.
[ExecuteAlways]                                       // also runs in the editor, outside Play mode
[DisallowMultipleComponent]                           // only one TestBasin per object
public class TestBasin : MonoBehaviour
{
    [Header("Inner cavity [m]")]                      // Inspector section title
    public float length = 20f;                        // inner length (Z axis)
    public float width = 12f;                         // inner width (X axis)
    public float depth = 6f;                          // inner depth (Y axis)
    public float wallThickness = 0.5f;                // floor and wall thickness
    public float waterLevel = 5.5f;                   // water height measured from the floor

    [Header("Materials")]                             // Inspector section title
    public Material basinMaterial;                    // floor and wall material (grid or plain)
    public Material waterSurfaceMaterial;             // water surface material
    public Material waterVolumeMaterial;              // optional blue volume material
    public bool showWaterVolume = false;              // shows/hides the transparent blue volume

    // Fixed names of the generated children (used to reuse objects instead of recreating them)
    const string FloorName = "Floor";                 // floor name
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

    [SerializeField, HideInInspector] private BoxCollider waterCollider;           // water trigger reference
    private Matrix4x4 lastGridMatrix;                                               // last matrix sent to the shader

    // Water volume trigger collider, used by SimpleBuoyancy
    public BoxCollider WaterCollider => waterCollider;                              // read-only access

    // World Y of the water surface
    public float WaterSurfaceY => transform.TransformPoint(0f, waterLevel, 0f).y;   // converts the local height to world space

    // Makes sure the basin exists and sends the grid matrix when the component is enabled
    void OnEnable()
    {
        if (transform.Find(FloorName) == null) Generate();  // generate if no pieces exist yet
        else ApplyMaterials();                              // otherwise just reapply the materials (useful on prefab instances)
        UpdateGridMatrix(true);                             // publish the grid matrix to the shader
    }

    // Keeps the grid aligned when the basin is moved (in the editor and in Play mode)
    void LateUpdate()
    {
        UpdateGridMatrix(false);                            // only resend if the matrix changed
    }

#if UNITY_EDITOR                                            // editor-only block
    // Regenerates the basin when a parameter changes in the Inspector
    void OnValidate()
    {
        EditorApplication.delayCall -= DelayedGenerate;     // avoid scheduling twice
        EditorApplication.delayCall += DelayedGenerate;     // deferred: Unity does not allow creating objects inside OnValidate
    }

    // Runs right after OnValidate, when the hierarchy can be modified
    void DelayedGenerate()
    {
        if (this == null) return;                                    // the component may have been destroyed meanwhile
        if (PrefabUtility.IsPartOfPrefabAsset(this)) return;         // never touch the prefab asset directly
        Generate();                                                  // regenerate with the new parameters
    }
#endif                                                               // end of editor-only block

    // Creates or updates every piece of the basin from the parameters
    [ContextMenu("Generate basin")]                                  // item in the component's ⋮ menu
    public void Generate()
    {
        float t = Mathf.Max(0.01f, wallThickness);                   // minimum thickness to avoid degenerate boxes
        float d = Mathf.Max(0.1f, depth);                            // minimum depth
        float outerX = width + 2f * t;                               // outer width (includes both walls)
        float outerZ = length + 2f * t;                              // outer length (includes both walls)
        float level = Mathf.Clamp(waterLevel, 0.01f, d);             // water level limited to the depth

        // Floor: top at y = 0, covers the whole outer area
        Piece(FloorName, new Vector3(0f, -t / 2f, 0f), new Vector3(outerX, t, outerZ));
        // Side walls (±X): span the whole outer length, covering the corners
        Piece(WallLeftName, new Vector3(-(width + t) / 2f, d / 2f, 0f), new Vector3(t, d, outerZ));
        Piece(WallRightName, new Vector3((width + t) / 2f, d / 2f, 0f), new Vector3(t, d, outerZ));
        // Front (+Z) and back (-Z) walls: fit between the side walls
        Piece(WallFrontName, new Vector3(0f, d / 2f, (length + t) / 2f), new Vector3(width, d, t));
        Piece(WallBackName, new Vector3(0f, d / 2f, -(length + t) / 2f), new Vector3(width, d, t));

        // Physical water volume: trigger from the floor (y = 0) up to the water level
        GameObject volume = Child(WaterVolumeName, null);                    // empty object (no mesh)
        volume.transform.localPosition = new Vector3(0f, level / 2f, 0f);    // centered halfway up the water column
        volume.transform.localRotation = Quaternion.identity;                // no rotation
        volume.transform.localScale = Vector3.one;                           // scale 1: the size lives in the collider
        volume.layer = 2;                                                    // "Ignore Raycast" layer: does not block sensor rays
        waterCollider = volume.GetComponent<BoxCollider>();                  // look for an existing collider
        if (waterCollider == null) waterCollider = volume.AddComponent<BoxCollider>(); // create it if missing
        waterCollider.isTrigger = true;                                      // trigger: detects entry, no collision
        waterCollider.center = Vector3.zero;                                 // centered on the object
        waterCollider.size = new Vector3(width, level, length);              // cavity size up to the water level

        // Water surface: Quad lying flat (facing up) at the water level
        GameObject surface = Child(WaterSurfaceName, PrimitiveType.Quad);    // Quad primitive
        RemoveCollider(surface);                                             // the surface is visual only
        surface.transform.localPosition = new Vector3(0f, level, 0f);        // at the water height
        surface.transform.localRotation = Quaternion.Euler(90f, 0f, 0f);     // rotate the Quad to lie horizontally
        surface.transform.localScale = new Vector3(width, length, 1f);       // X = width, Quad Y = length

        // Optional visual volume: very transparent blue box filling the water column
        GameObject tint = Child(WaterTintName, PrimitiveType.Cube);          // cube primitive
        RemoveCollider(tint);                                                // visual only
        tint.transform.localPosition = new Vector3(0f, level / 2f, 0f);      // center of the water column
        tint.transform.localRotation = Quaternion.identity;                  // no rotation
        tint.transform.localScale = new Vector3(width - 0.01f, level - 0.01f, length - 0.01f); // slightly smaller than the cavity (avoids z-fighting)
        tint.SetActive(showWaterVolume);                                     // visible only if the option is on

        ApplyMaterials();                                                    // apply the materials to the pieces
        UpdateGridMatrix(true);                                              // update the grid with the new dimensions
    }

    // Applies the configured materials to the generated pieces
    public void ApplyMaterials()
    {
        foreach (string n in new[] { FloorName, WallLeftName, WallRightName, WallFrontName, WallBackName }) // floor and walls
            SetMaterial(n, basinMaterial);                                    // basin material
        SetMaterial(WaterSurfaceName, waterSurfaceMaterial);                  // surface material
        SetMaterial(WaterTintName, waterVolumeMaterial);                      // visual volume material
        Transform tint = transform.Find(WaterTintName);                       // find the visual volume
        if (tint != null) tint.gameObject.SetActive(showWaterVolume);         // sync its visibility with the option
    }

    // Creates/updates a solid piece (static cube with a BoxCollider)
    void Piece(string pieceName, Vector3 center, Vector3 size)
    {
        GameObject go = Child(pieceName, PrimitiveType.Cube);                 // cube primitive (already has a BoxCollider)
        go.transform.localPosition = center;                                  // piece center
        go.transform.localRotation = Quaternion.identity;                     // no rotation
        go.transform.localScale = size;                                       // the unit cube becomes a box of the right size
        if (go.GetComponent<BoxCollider>() == null) go.AddComponent<BoxCollider>(); // make sure the BoxCollider exists
#if UNITY_EDITOR                                                              // static flags only exist in the editor
        GameObjectUtility.SetStaticEditorFlags(go, StaticEditorFlags.BatchingStatic
                                                   | StaticEditorFlags.OccluderStatic | StaticEditorFlags.OccludeeStatic
                                                   | StaticEditorFlags.ReflectionProbeStatic); // mark as static
#endif                                                                        // end of editor-only block
    }

    // Finds a child by name; creates it (primitive or empty) if it does not exist
    GameObject Child(string childName, PrimitiveType? primitive)
    {
        Transform existing = transform.Find(childName);                       // try to reuse the existing child
        if (existing != null) return existing.gameObject;                     // found: return it
        GameObject go = primitive.HasValue ? GameObject.CreatePrimitive(primitive.Value) : new GameObject(); // create
        go.name = childName;                                                  // set the fixed name
        go.transform.SetParent(transform, false);                             // parent to the basin, keeping the local position
        return go;                                                            // return the new object
    }

    // Removes the collider of a visual-only object
    static void RemoveCollider(GameObject go)
    {
        Collider c = go.GetComponent<Collider>();                             // look for the collider
        if (c == null) return;                                                // nothing to remove
        if (Application.isPlaying) Destroy(c);                                // in Play mode: normal removal
        else DestroyImmediate(c);                                             // in the editor: immediate removal
    }

    // Applies a material to a child's Renderer, if both exist
    void SetMaterial(string childName, Material mat)
    {
        Transform child = transform.Find(childName);                          // find the child
        if (child == null || mat == null) return;                             // nothing to do
        Renderer r = child.GetComponent<Renderer>();                          // get the Renderer
        if (r != null && r.sharedMaterial != mat) r.sharedMaterial = mat;     // swap only if different
    }

    // Sends the shader the matrix from world space to grid space (basin local space + alignment)
    void UpdateGridMatrix(bool force)
    {
        // Offset that puts the inner edges on integer lines when width/length are odd
        Vector3 align = new Vector3(Mathf.Repeat(-width / 2f, 1f), 0f, Mathf.Repeat(-length / 2f, 1f));
        Matrix4x4 m = Matrix4x4.Translate(align) * transform.worldToLocalMatrix; // world → basin local → aligned
        if (!force && m == lastGridMatrix) return;                            // nothing changed: skip the work
        lastGridMatrix = m;                                                   // store for the next comparison
        Shader.SetGlobalMatrix(GridMatrixId, m);                              // publish the matrix to every shader
        Shader.SetGlobalFloat(GridValidId, 1f);                               // tell the shader the matrix is valid
    }

    // Draws the cavity and water outlines in the Scene view
    void OnDrawGizmosSelected()
    {
        Gizmos.matrix = transform.localToWorldMatrix;                         // draw in the basin's local space
        Gizmos.color = Color.yellow;                                          // cavity color
        Gizmos.DrawWireCube(new Vector3(0f, depth / 2f, 0f), new Vector3(width, depth, length)); // inner cavity
        Gizmos.color = Color.cyan;                                            // water color
        Gizmos.DrawWireCube(new Vector3(0f, waterLevel / 2f, 0f), new Vector3(width, waterLevel, length)); // water column
    }
}
