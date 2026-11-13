using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

// Builds Assets/AUV/Prefabs/RobotFishDetailed.prefab from the CAD meshes converted by
// Tools/cad_analysis/cad_to_unity.py (Assets/AUV/Models/RobotFish: one OBJ per part + robot_fish_parts.json).
//
// Menu: Tools → Robot Fish → Build Detailed Prefab. Run it again after converting a new version of the CAD.
//
// The physics and the ROS components are copied from the simple RobotFish prefab, so both prefabs behave
// the same; only the visual changes:
//   Visual/Hull, Camera, Electronics, PitchMechanism, TailDrive, DorsalFin   CAD parts, grouped by function
//   Visual/PitchMechanism/MovingMass                                         carriage + metal weight on the tube axis, moved by RobotFishVisual
//   Visual/TailDrive/TailPivot                                               hub, arm and fin, turned by RobotFishVisual
//   Visual/NotInCad                                                          sensors that are not in the CAD
// The CAD sliding mechanism (HS-5496MH servo, horn, rod, carriage) is shown at its CAD pose, not moved: how the
// real moving mass is driven is not known.
// The tube and the dome are transparent so that the inside can be seen. All visuals are on the "Robot" layer,
// which the robot's own camera does not render (it would otherwise see the dome and the parts around it).
public static class RobotFishPrefabBuilder
{
    const string ModelFolder = "Assets/AUV/Models/RobotFish";
    const string MaterialFolder = "Assets/AUV/Materials/RobotFishDetailed";
    const string SimplePrefab = "Assets/AUV/Prefabs/RobotFish.prefab";
    const string DetailedPrefab = "Assets/AUV/Prefabs/RobotFishDetailed.prefab";
    const string RobotLayerName = "Robot";

    // ---- manifest written by cad_to_unity.py
    [System.Serializable] class Part { public string name, file, group, moving, material; public float[] pivot, color; public int triangles; }
    [System.Serializable] class Mechanisms { public float[] shaft, hornPinAtFrontStop, carriagePinAtFrontStop, tailAxis; public float hornRadius, rodLength, cadFinDeflectionDeg; }
    [System.Serializable] class Manifest { public Part[] parts; public Mechanisms mechanisms; }

    static Vector3 V(float[] a) => new Vector3(a[0], a[1], a[2]);

    [MenuItem("Tools/Robot Fish/Build Detailed Prefab")]
    public static void Build()
    {
        var manifest = JsonUtility.FromJson<Manifest>(File.ReadAllText(Path.Combine(ModelFolder, "robot_fish_parts.json")));
        int robotLayer = EnsureLayer(RobotLayerName);
        ConfigureImporters(manifest);
        var materials = new Dictionary<string, Material>();

        // Start from the simple prefab (physics, collider, sensors, ROS subscriber) and replace its visual
        GameObject root = PrefabUtility.LoadPrefabContents(SimplePrefab);
        root.name = "RobotFishDetailed";
        Object.DestroyImmediate(root.transform.Find("Visual").gameObject);
        var physics = root.GetComponent<RobotFishPhysics>();
        physics.tailFinPivot = null;                                  // the tail is moved by RobotFishVisual

        Transform visual = Child(root.transform, "Visual", Vector3.zero);
        var groups = new Dictionary<string, Transform>();
        Mechanisms m = manifest.mechanisms;
        Transform tailPivot = null;
        foreach (Part p in manifest.parts)
        {
            if (!groups.TryGetValue(p.group, out Transform group)) groups[p.group] = group = Child(visual, p.group, Vector3.zero);
            Transform parent = group;
            Vector3 position = V(p.pivot);
            if (p.moving == "tail")                                   // all tail parts share one pivot
            {
                if (tailPivot == null) tailPivot = Child(group, "TailPivot", V(m.tailAxis));
                parent = tailPivot;
                position = Vector3.zero;
            }
            PartObject(p, parent, position, Mat(p, materials));     // CAD mechanism parts stay at their CAD pose
        }

        // ---- moving mass: square carriage with a metal weight on top (like on the real robot), on the tube axis.
        //      Fixed size (the weight set in RobotFishPhysics only matters for the physics); moved by RobotFishVisual.
        Transform movingMass = Child(groups["PitchMechanism"], "MovingMass", Vector3.zero);
        Material carriageMat = Lit("MovingMassCarriage", new Color(0.88f, 0.52f, 0.18f), 0.4f);
        Material steel = Lit("MovingMassWeight", new Color(0.62f, 0.65f, 0.70f), 0.75f);
        steel.SetFloat("_Metallic", 0.8f);
        Prim(PrimitiveType.Cube, "Carriage", movingMass, new Vector3(0f, -0.012f, 0f), Vector3.zero, new Vector3(0.05f, 0.008f, 0.05f), carriageMat);
        Prim(PrimitiveType.Cube, "MetalWeight", movingMass, new Vector3(0f, -0.0005f, 0f), Vector3.zero, new Vector3(0.04f, 0.015f, 0.04f), steel);

        // ---- parts that are not in the CAD (estimates, see Docs/RobotFish_Assumptions.md)
        Transform notInCad = Child(visual, "NotInCad", Vector3.zero);
        Material estimate = Lit("Estimate", new Color(1f, 0.85f, 0.2f), 0.4f);
        // pressure sensor on the rear cap and GNSS antenna on top: real positions unknown
        Prim(PrimitiveType.Cylinder, "PressureSensor (position estimate)", notInCad, new Vector3(0.218f, 0.03f, 0f), new Vector3(0f, 0f, 90f), new Vector3(0.016f, 0.01f, 0.016f), estimate);
        Prim(PrimitiveType.Cylinder, "GNSSAntenna (position estimate)", notInCad, new Vector3(0f, 0.065f, 0f), Vector3.zero, new Vector3(0.03f, 0.005f, 0.03f), estimate);
        Prim(PrimitiveType.Cylinder, "GNSSMast (position estimate)", notInCad, new Vector3(0f, 0.057f, 0f), Vector3.zero, new Vector3(0.008f, 0.004f, 0.008f), estimate);

        // ---- mechanism animation
        var anim = root.AddComponent<RobotFishVisual>();
        anim.physics = physics;
        anim.movingMass = movingMass;
        anim.tailPivot = tailPivot;
        anim.tailAxis = V(m.tailAxis);

        // ---- layers: the robot camera must not see the robot itself
        foreach (Transform t in visual.GetComponentsInChildren<Transform>(true)) t.gameObject.layer = robotLayer;
        foreach (Camera cam in root.GetComponentsInChildren<Camera>(true)) cam.cullingMask &= ~(1 << robotLayer);

        PrefabUtility.SaveAsPrefabAsset(root, DetailedPrefab);
        PrefabUtility.UnloadPrefabContents(root);
        AssetDatabase.SaveAssets();
        Debug.Log($"[RobotFishPrefabBuilder] {DetailedPrefab} built ({manifest.parts.Length} CAD parts)");
    }

    [MenuItem("Tools/Robot Fish/Use Detailed Prefab In Basin Scenes")]
    public static void UseInBasinScenes()
    {
        foreach (string path in new[] { "Assets/TestBasin/Scenes/Basin_Grid.unity", "Assets/TestBasin/Scenes/Basin_NoGrid.unity" })
        {
            var scene = EditorSceneManager.OpenScene(path, OpenSceneMode.Single);
            var old = Object.FindAnyObjectByType<RobotFishPhysics>().gameObject;
            var fish = (GameObject)PrefabUtility.InstantiatePrefab(AssetDatabase.LoadAssetAtPath<GameObject>(DetailedPrefab), scene);
            fish.transform.SetPositionAndRotation(old.transform.position, old.transform.rotation);
            var oldLog = old.GetComponent<GroundTruthLogger>();
            if (oldLog != null) EditorUtility.CopySerialized(oldLog, fish.AddComponent<GroundTruthLogger>());
            var cam = Object.FindAnyObjectByType<SimulationCamera>();
            if (cam != null) cam.target = fish.transform;
            Object.DestroyImmediate(old);
            EditorSceneManager.SaveScene(scene);
            Debug.Log($"[RobotFishPrefabBuilder] {path} uses RobotFishDetailed");
        }
    }

    // OBJ import: no materials (they are created here), normals computed with a smoothing angle
    static void ConfigureImporters(Manifest manifest)
    {
        foreach (Part p in manifest.parts)
        {
            var importer = (ModelImporter)AssetImporter.GetAtPath($"{ModelFolder}/{p.file}");
            importer.materialImportMode = ModelImporterMaterialImportMode.None;
            importer.importNormals = ModelImporterNormals.Calculate;
            importer.normalSmoothingAngle = 40f;
            importer.isReadable = false;
            importer.SaveAndReimport();
        }
    }

    // One GameObject with the mesh of a CAD part
    static GameObject PartObject(Part p, Transform parent, Vector3 localPosition, Material mat)
    {
        Mesh mesh = AssetDatabase.LoadAllAssetsAtPath($"{ModelFolder}/{p.file}").OfType<Mesh>().First();
        var go = new GameObject(p.name);
        go.transform.SetParent(parent, false);
        go.transform.localPosition = localPosition;
        go.AddComponent<MeshFilter>().sharedMesh = mesh;
        go.AddComponent<MeshRenderer>().sharedMaterial = mat;
        return go;
    }

    // Material of a part: transparent acrylic, or the color of the part in the CAD
    static Material Mat(Part p, Dictionary<string, Material> cache)
    {
        if (p.material == "acrylic") return Acrylic();
        var c = new Color(p.color[0], p.color[1], p.color[2], 1f);
        string key = ColorUtility.ToHtmlStringRGB(c);
        if (!cache.TryGetValue(key, out Material mat)) cache[key] = mat = Lit("CAD_" + key, c, 0.35f);
        return mat;
    }

    static Material Lit(string name, Color color, float smoothness)
    {
        string path = $"{MaterialFolder}/{name}.mat";
        if (!AssetDatabase.IsValidFolder(MaterialFolder)) AssetDatabase.CreateFolder("Assets/AUV/Materials", "RobotFishDetailed");
        var mat = AssetDatabase.LoadAssetAtPath<Material>(path);
        if (mat == null) { mat = new Material(Shader.Find("Universal Render Pipeline/Lit")); AssetDatabase.CreateAsset(mat, path); }
        mat.SetColor("_BaseColor", color);
        mat.SetFloat("_Smoothness", smoothness);
        EditorUtility.SetDirty(mat);
        return mat;
    }

    // URP Lit, transparent (alpha blending), for the acrylic tube and dome
    static Material Acrylic()
    {
        Material mat = Lit("Acrylic", new Color(0.85f, 0.93f, 1f, 0.18f), 0.9f);
        mat.SetFloat("_Surface", 1f);                                 // transparent
        mat.SetFloat("_Blend", 0f);                                   // alpha
        mat.SetFloat("_SrcBlend", (float)UnityEngine.Rendering.BlendMode.SrcAlpha);
        mat.SetFloat("_DstBlend", (float)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
        mat.SetFloat("_ZWrite", 0f);
        mat.EnableKeyword("_SURFACE_TYPE_TRANSPARENT");
        mat.renderQueue = (int)UnityEngine.Rendering.RenderQueue.Transparent;
        mat.SetOverrideTag("RenderType", "Transparent");
        EditorUtility.SetDirty(mat);
        return mat;
    }

    static Transform Child(Transform parent, string name, Vector3 localPosition)
    {
        var t = new GameObject(name).transform;
        t.SetParent(parent, false);
        t.localPosition = localPosition;
        return t;
    }

    static void Prim(PrimitiveType type, string name, Transform parent, Vector3 pos, Vector3 euler, Vector3 scale, Material mat)
    {
        GameObject go = GameObject.CreatePrimitive(type);
        go.name = name;
        Object.DestroyImmediate(go.GetComponent<Collider>());
        go.transform.SetParent(parent, false);
        go.transform.localPosition = pos;
        go.transform.localEulerAngles = euler;
        go.transform.localScale = scale;
        go.GetComponent<MeshRenderer>().sharedMaterial = mat;
    }

    // Adds a user layer to Project Settings → Tags and Layers if it does not exist yet
    static int EnsureLayer(string name)
    {
        int existing = LayerMask.NameToLayer(name);
        if (existing >= 0) return existing;
        var tagManager = new SerializedObject(AssetDatabase.LoadAllAssetsAtPath("ProjectSettings/TagManager.asset")[0]);
        SerializedProperty layers = tagManager.FindProperty("layers");
        for (int i = 8; i < layers.arraySize; i++)
        {
            SerializedProperty layer = layers.GetArrayElementAtIndex(i);
            if (string.IsNullOrEmpty(layer.stringValue))
            {
                layer.stringValue = name;
                tagManager.ApplyModifiedProperties();
                return i;
            }
        }
        throw new System.Exception("No free user layer for " + name);
    }
}
