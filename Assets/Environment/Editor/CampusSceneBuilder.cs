using System.Collections.Generic;                     // Dictionary, List
using UnityEditor;                                    // editor API
using UnityEditor.SceneManagement;                    // scenes
using UnityEngine;                                    // core API
using UnityEngine.Rendering;                          // Volume, CullMode
using UnityEngine.Rendering.Universal;                // URP post-processing and camera data

// Builds Assets/Environment/Scenes/AirSeaTestBasin_Campus.unity: the Air-Sea Test Basin building of ENSTA (Brest
// campus) around the existing basin, with its surroundings (parking lot, road, trees, cars, fence), a sky, the
// lighting and a camera tour, for presentations. Menu: Tools → Campus → Build Campus Scene.
//
// The scene starts from Basin_NoGrid (basin, robot fish, sensors, cameras) and adds everything else. Every shape
// is made of simple primitives or small generated meshes (low-poly style), so the scene stays light and can be
// rebuilt at any time; dimensions are the constants below (estimated from photos of the building and from
// https://www.ensta.fr/actualites/inauguration-du-bassin-dexperimentation-air-mer-unique-en-europe, see
// Docs/Campus_Scene.md).
//
// Frame: the basin is centered on the origin, its long axis along Z; its rim (y = 6) is the floor of the hall.
// The street level is 3 m lower (the entrance ramp climbs 3 m, as on the photos).
public static class CampusSceneBuilder
{
    const string SourceScene = "Assets/TestBasin/Scenes/Basin_NoGrid.unity";
    const string ScenePath = "Assets/Environment/Scenes/AirSeaTestBasin_Campus.unity";
    const string MaterialFolder = "Assets/Environment/Materials";
    const string TextureFolder = "Assets/Environment/Textures";

    // ---- building (estimates, see Docs/Campus_Scene.md)
    const float HallLength = 36f;        // along Z [m]
    const float HallWidth = 22f;         // along X [m]
    const float Ground = 3f;             // street level (the hall floor is the basin rim, y = 6)
    const float HallFloor = 6f;          // hall floor = basin rim
    const float ConcreteTop = 6.8f;      // top of the grey concrete base (3.8 m above the street)
    const float Eave = 12.8f;            // top of the white walls (9.8 m above the street)
    const float RoofRise = 2.8f;         // height of the canvas domes and of the arched end walls
    const int DomesPerRow = 7;           // two rows of inflated canvas domes
    const float WallThickness = 0.3f;
    const float DoorZ = 11f, DoorWidth = 5f, DoorTop = 10.5f;   // entrance (east wall)
    const float AviaryHeight = 5f;       // netted aviary over the basin (5 m, per the ENSTA page)
    const float BasinHalfX = 6.5f, BasinHalfZ = 10.5f;          // outer half-size of the basin walls

    static readonly Dictionary<string, Material> materials = new Dictionary<string, Material>();
    static readonly System.Random rng = new System.Random(7);   // fixed seed: same scene at every build

    [MenuItem("Tools/Campus/Build Campus Scene")]
    public static void Build()
    {
        materials.Clear();
        meshCount = 0;
        ConfigureTextures();
        if (!AssetDatabase.IsValidFolder("Assets/Environment/Scenes")) AssetDatabase.CreateFolder("Assets/Environment", "Scenes");
        var scene = EditorSceneManager.OpenScene(SourceScene, OpenSceneMode.Single);
        EditorSceneManager.SaveScene(scene, ScenePath, true);
        scene = EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);

        Transform campus = new GameObject("Campus").transform;
        BuildGround(Child(campus, "Ground"));
        BuildRoadsAndParking(Child(campus, "RoadsAndParking"));
        BuildHall(Child(campus, "AirSeaTestBasin"));
        BuildInterior(Child(campus, "HallInterior"));
        BuildRamp(Child(campus, "EntranceRamp"));
        BuildNeighbours(Child(campus, "NeighbourBuildings"));
        BuildFence(Child(campus, "Fence"));
        BuildTrees(Child(campus, "Trees"));
        BuildCars(Child(campus, "Cars"));
        BuildStreetFurniture(Child(campus, "StreetFurniture"));
        foreach (Transform t in campus.GetComponentsInChildren<Transform>(true))
            GameObjectUtility.SetStaticEditorFlags(t.gameObject, StaticEditorFlags.BatchingStatic | StaticEditorFlags.ContributeGI | StaticEditorFlags.OccluderStatic);

        SetupLighting();
        SetupCamera();
        SetupRobot();
        EditorSceneManager.SaveScene(scene);
        AddToBuildSettings();
        Debug.Log("[CampusSceneBuilder] " + ScenePath + " built");
    }

    // ================================================================== ground, roads, parking
    static void BuildGround(Transform p)
    {
        // grass all around, with a hole under the building (the basin goes below the street level)
        Material grass = Mat("Grass", new Color(0.36f, 0.52f, 0.24f), 0.1f);
        float half = 400f, hx = HallWidth / 2f, hz = HallLength / 2f;
        Box("Grass_W", p, new Vector3(-(half + hx) / 2f, Ground - 0.5f, 0f), new Vector3(half - hx, 1f, 2f * half), grass);
        Box("Grass_E", p, new Vector3((half + hx) / 2f, Ground - 0.5f, 0f), new Vector3(half - hx, 1f, 2f * half), grass);
        Box("Grass_S", p, new Vector3(0f, Ground - 0.5f, -(half + hz) / 2f), new Vector3(2f * hx, 1f, half - hz), grass);
        Box("Grass_N", p, new Vector3(0f, Ground - 0.5f, (half + hz) / 2f), new Vector3(2f * hx, 1f, half - hz), grass);
    }

    static void BuildRoadsAndParking(Transform p)
    {
        Material asphalt = Mat("Asphalt", new Color(0.23f, 0.24f, 0.25f), 0.15f);
        Material sidewalk = Mat("Sidewalk", new Color(0.72f, 0.72f, 0.70f), 0.1f);
        Material paint = Mat("RoadPaint", new Color(0.92f, 0.92f, 0.9f), 0.2f);
        float y = Ground + 0.01f;
        // road along the west wall, sidewalk between road and building
        Box("Road", p, new Vector3(-20f, y, 0f), new Vector3(8f, 0.02f, 240f), asphalt);
        for (float z = -118f; z < 118f; z += 6f) Box("RoadLine", p, new Vector3(-20f, y + 0.012f, z), new Vector3(0.15f, 0.01f, 3f), paint);
        Box("Sidewalk_W", p, new Vector3(-13.5f, y + 0.08f, 0f), new Vector3(5f, 0.15f, 240f), sidewalk);
        Box("Sidewalk_E", p, new Vector3(-25f, y + 0.08f, 0f), new Vector3(2f, 0.15f, 240f), sidewalk);
        // crosswalk to the parking lot
        for (int i = 0; i < 7; i++) Box("Crosswalk", p, new Vector3(-23.2f + i * 1.07f, y + 0.012f, -26f), new Vector3(0.55f, 0.01f, 3.5f), paint);
        // parking lot west of the road, three rows of stalls
        Box("Parking", p, new Vector3(-43f, y, -2f), new Vector3(32f, 0.02f, 66f), asphalt);
        foreach (float x in new[] { -30.5f, -43f, -55.5f })
        {
            for (int i = 0; i <= 24; i++) Box("StallLine", p, new Vector3(x, y + 0.012f, -33f + i * 2.6f), new Vector3(5f, 0.01f, 0.12f), paint);
        }
        // service road to the building entrance (east side)
        Box("ServiceRoad", p, new Vector3(0f, y, -26f), new Vector3(60f, 0.02f, 7f), asphalt);
        Box("ServiceYard", p, new Vector3(20f, y, -6f), new Vector3(10f, 0.02f, 34f), asphalt);
    }

    // ================================================================== building
    static void BuildHall(Transform p)
    {
        Material concrete = Mat("Concrete", new Color(0.62f, 0.63f, 0.64f), 0.15f);
        Material cladding = Mat("WhiteCladding", new Color(0.93f, 0.94f, 0.95f), 0.35f);
        Material canvas = Mat("CanvasMembrane", new Color(0.97f, 0.97f, 0.98f), 0.25f);
        Material darkPanel = Mat("DarkPanel", new Color(0.30f, 0.31f, 0.33f), 0.3f);
        Material vent = Mat("Louvre", new Color(0.78f, 0.79f, 0.80f), 0.3f, 0.4f);
        float hx = HallWidth / 2f, hz = HallLength / 2f, t = WallThickness;
        float wx = hx - t / 2f, wz = hz - t / 2f;
        float dz0 = DoorZ - DoorWidth / 2f, dz1 = DoorZ + DoorWidth / 2f;

        // concrete base: full height up to the hall floor, then a band up to ConcreteTop (open at the door)
        Transform baseT = Child(p, "ConcreteBase");
        Box("Base_W", baseT, new Vector3(-wx, (Ground + ConcreteTop) / 2f, 0f), new Vector3(t, ConcreteTop - Ground, HallLength), concrete);
        Box("Base_S", baseT, new Vector3(0f, (Ground + ConcreteTop) / 2f, -wz), new Vector3(HallWidth, ConcreteTop - Ground, t), concrete);
        Box("Base_N", baseT, new Vector3(0f, (Ground + ConcreteTop) / 2f, wz), new Vector3(HallWidth, ConcreteTop - Ground, t), concrete);
        Box("Base_E_low", baseT, new Vector3(wx, (Ground + HallFloor) / 2f, 0f), new Vector3(t, HallFloor - Ground, HallLength), concrete);
        SegmentZ("Base_E_band", baseT, wx, HallFloor, ConcreteTop, -hz, dz0, dz1, hz, concrete);
        // dark service doors and louvres in the concrete base (west wall, as on the photos)
        Box("ServiceDoor_1", baseT, new Vector3(-hx - 0.02f, Ground + 1.8f, 6f), new Vector3(0.05f, 3.4f, 3f), darkPanel);
        Box("ServiceDoor_2", baseT, new Vector3(-hx - 0.02f, Ground + 1.8f, 9.2f), new Vector3(0.05f, 3.4f, 3f), darkPanel);
        Box("BaseLouvre_1", baseT, new Vector3(-hx - 0.03f, Ground + 1.3f, -8f), new Vector3(0.05f, 0.7f, 0.9f), vent);
        Box("BaseLouvre_2", baseT, new Vector3(-hx - 0.03f, Ground + 1.3f, -6.5f), new Vector3(0.05f, 0.7f, 0.9f), vent);

        // white cladding up to the eaves (east wall open for the entrance door)
        Transform walls = Child(p, "WhiteWalls");
        Box("Wall_W", walls, new Vector3(-wx, (ConcreteTop + Eave) / 2f, 0f), new Vector3(t, Eave - ConcreteTop, HallLength), cladding);
        SegmentZ("Wall_E", walls, wx, ConcreteTop, Eave, -hz, dz0, dz1, hz, cladding);
        Box("Wall_E_overDoor", walls, new Vector3(wx, (DoorTop + Eave) / 2f, DoorZ), new Vector3(t, Eave - DoorTop, DoorWidth), cladding);
        foreach (float sz in new[] { -1f, 1f }) EndWall(walls, sz * wz, cladding);
        for (int i = 0; i < 3; i++)                                   // high louvres on the long walls
        {
            Box("Louvre_W", walls, new Vector3(-hx - 0.03f, Eave - 1.4f, -10f + i * 10f), new Vector3(0.05f, 1.1f, 1.2f), vent);
            Box("Louvre_E", walls, new Vector3(hx + 0.03f, Eave - 1.4f, -12f + i * 9f), new Vector3(0.05f, 1.1f, 1.2f), vent);
        }
        Box("Louvre_S", walls, new Vector3(-7f, Eave - 2f, -hz - 0.03f), new Vector3(1.2f, 2.2f, 0.05f), vent);
        Box("EaveTrim_W", walls, new Vector3(-hx, Eave + 0.1f, 0f), new Vector3(0.45f, 0.25f, HallLength + 0.4f), cladding);
        Box("EaveTrim_E", walls, new Vector3(hx, Eave + 0.1f, 0f), new Vector3(0.45f, 0.25f, HallLength + 0.4f), cladding);

        // roof: flat deck covered by two rows of inflated canvas domes
        Transform roof = Child(p, "CanvasRoof");
        Box("RoofDeck", roof, new Vector3(0f, Eave + 0.1f, 0f), new Vector3(HallWidth, 0.2f, HallLength), canvas);
        Mesh dome = Hemisphere(24, 10);
        float pitch = HallLength / DomesPerRow;
        for (int row = 0; row < 2; row++)
            for (int i = 0; i < DomesPerRow; i++)
            {
                var go = MeshObject("Dome", roof, dome, canvas);
                go.transform.localPosition = new Vector3((row == 0 ? -1f : 1f) * HallWidth / 4f, Eave + 0.2f, -hz + pitch * (i + 0.5f));
                go.transform.localScale = new Vector3(HallWidth / 4f + 0.25f, RoofRise, pitch / 2f + 0.35f);
            }

        // entrance: translucent polycarbonate door (slid open), frame, sign
        Transform entrance = Child(p, "Entrance");
        Material polycarbonate = Transparent("Polycarbonate", new Color(0.88f, 0.92f, 0.95f, 0.55f), 0.7f);
        Material frame = Mat("DoorFrame", new Color(0.55f, 0.57f, 0.6f), 0.5f, 0.6f);
        Box("DoorFrame_Top", entrance, new Vector3(hx + 0.05f, DoorTop + 0.1f, DoorZ), new Vector3(0.12f, 0.2f, DoorWidth + 0.4f), frame);
        Box("DoorFrame_L", entrance, new Vector3(hx + 0.05f, (HallFloor + DoorTop) / 2f, dz0 - 0.1f), new Vector3(0.12f, DoorTop - HallFloor, 0.2f), frame);
        Box("DoorFrame_R", entrance, new Vector3(hx + 0.05f, (HallFloor + DoorTop) / 2f, dz1 + 0.1f), new Vector3(0.12f, DoorTop - HallFloor, 0.2f), frame);
        Box("SlidingDoor (open)", entrance, new Vector3(hx + 0.18f, (HallFloor + DoorTop) / 2f, dz0 - DoorWidth / 2f + 0.3f), new Vector3(0.08f, DoorTop - HallFloor, DoorWidth), polycarbonate);
        for (int i = 1; i < 5; i++)                                   // horizontal ribs of the door panel
            Box("DoorRib", entrance, new Vector3(hx + 0.23f, HallFloor + i * (DoorTop - HallFloor) / 5f, dz0 - DoorWidth / 2f + 0.3f), new Vector3(0.03f, 0.05f, DoorWidth), frame);
        Box("DoorCanopy", entrance, new Vector3(hx + 0.7f, DoorTop + 0.6f, DoorZ), new Vector3(1.4f, 0.12f, DoorWidth + 1.2f), cladding);

        // logos and signs (decals from Tools/scene_assets/make_textures.py)
        Transform signs = Child(p, "LogosAndSigns");
        Decal("BasinSign", signs, "basin_sign", new Vector3(hx + 0.04f, Eave - 1.6f, 13.5f), Vector3.right, 7.5f);
        Decal("EnstaLogo_W", signs, "ensta_logo_stacked", new Vector3(-hx - 0.04f, Eave - 3.1f, 2f), Vector3.left, 6.5f);
        Decal("Slogan_W", signs, "ensta_slogan", new Vector3(-hx - 0.04f, Eave - 2.8f, 12.5f), Vector3.left, 4.2f);
        Decal("EnstaLogo_S", signs, "ensta_logo_stacked", new Vector3(-7.5f, Eave - 2.2f, -hz - 0.04f), Vector3.back, 3.2f, 90f);
        Decal("EnstaLogo_N", signs, "ensta_logo_stacked", new Vector3(7.5f, Eave - 2.2f, hz + 0.04f), Vector3.forward, 3.2f, 90f);
    }

    // End wall with an arched top (the profile of the canvas roof), from the concrete band to the arch
    static void EndWall(Transform p, float z, Material mat)
    {
        var pts = new List<Vector3>();
        float hx = HallWidth / 2f;
        pts.Add(new Vector3(-hx, ConcreteTop, z));
        pts.Add(new Vector3(hx, ConcreteTop, z));
        for (int i = 0; i <= 16; i++)                                 // arch from +x to −x
        {
            float x = hx - i * HallWidth / 16f;
            pts.Add(new Vector3(x, Eave + RoofRise * Mathf.Sqrt(Mathf.Max(0f, 1f - x * x / (hx * hx))), z));
        }
        MeshObject(z < 0 ? "EndWall_S" : "EndWall_N", p, Prism(pts.ToArray(), new Vector3(0f, 0f, z < 0 ? WallThickness : -WallThickness)), mat);
    }

    // ================================================================== interior
    static void BuildInterior(Transform p)
    {
        Material floor = Mat("HallFloor", new Color(0.55f, 0.56f, 0.57f), 0.35f);
        Material steel = Mat("Steel", new Color(0.70f, 0.72f, 0.75f), 0.6f, 0.8f);
        Material yellow = Mat("SafetyYellow", new Color(0.95f, 0.75f, 0.1f), 0.4f);
        Material net = Transparent("AviaryNet", new Color(0.1f, 0.12f, 0.14f, 0.22f), 0.1f, true);
        float hx = HallWidth / 2f - WallThickness, hz = HallLength / 2f - WallThickness, y = HallFloor - 0.15f;

        // hall floor around the basin opening
        Box("Floor_W", p, new Vector3(-(hx + BasinHalfX) / 2f, y, 0f), new Vector3(hx - BasinHalfX, 0.3f, 2f * hz), floor);
        Box("Floor_E", p, new Vector3((hx + BasinHalfX) / 2f, y, 0f), new Vector3(hx - BasinHalfX, 0.3f, 2f * hz), floor);
        Box("Floor_S", p, new Vector3(0f, y, -(hz + BasinHalfZ) / 2f), new Vector3(2f * BasinHalfX, 0.3f, hz - BasinHalfZ), floor);
        Box("Floor_N", p, new Vector3(0f, y, (hz + BasinHalfZ) / 2f), new Vector3(2f * BasinHalfX, 0.3f, hz - BasinHalfZ), floor);
        // yellow safety edge and railing around the basin
        float ex = BasinHalfX + 0.15f, ez = BasinHalfZ + 0.15f;
        Box("Edge_W", p, new Vector3(-ex, HallFloor + 0.005f, 0f), new Vector3(0.3f, 0.01f, 2f * ez + 0.3f), yellow);
        Box("Edge_E", p, new Vector3(ex, HallFloor + 0.005f, 0f), new Vector3(0.3f, 0.01f, 2f * ez + 0.3f), yellow);
        Box("Edge_S", p, new Vector3(0f, HallFloor + 0.005f, -ez), new Vector3(2f * ex, 0.01f, 0.3f), yellow);
        Box("Edge_N", p, new Vector3(0f, HallFloor + 0.005f, ez), new Vector3(2f * ex, 0.01f, 0.3f), yellow);
        Transform rail = Child(p, "Railing");
        float rx = BasinHalfX + 0.6f, rz = BasinHalfZ + 0.6f;
        foreach (float h in new[] { 0.55f, 1.1f })
        {
            Box("Rail_W", rail, new Vector3(-rx, HallFloor + h, 0f), new Vector3(0.05f, 0.05f, 2f * rz), steel);
            Box("Rail_E", rail, new Vector3(rx, HallFloor + h, 0f), new Vector3(0.05f, 0.05f, 2f * rz), steel);
            Box("Rail_S", rail, new Vector3(0f, HallFloor + h, -rz), new Vector3(2f * rx, 0.05f, 0.05f), steel);
            Box("Rail_N", rail, new Vector3(0f, HallFloor + h, rz), new Vector3(2f * rx, 0.05f, 0.05f), steel);
        }
        for (float z = -rz; z <= rz + 0.01f; z += 2f * rz / 11f)
            foreach (float sx in new[] { -1f, 1f }) Box("Post", rail, new Vector3(sx * rx, HallFloor + 0.55f, z), new Vector3(0.06f, 1.1f, 0.06f), steel);
        for (float x = -rx; x <= rx + 0.01f; x += 2f * rx / 7f)
            foreach (float sz in new[] { -1f, 1f }) Box("Post", rail, new Vector3(x, HallFloor + 0.55f, sz * rz), new Vector3(0.06f, 1.1f, 0.06f), steel);

        // aviary: steel frame and net over the basin, 5 m high (for drone tests)
        Transform aviary = Child(p, "Aviary");
        float top = HallFloor + AviaryHeight, ax = BasinHalfX, az = BasinHalfZ;
        foreach (float sx in new[] { -1f, 1f })
            foreach (float z in new[] { -az, -az / 3f, az / 3f, az })
                Box("AviaryPost", aviary, new Vector3(sx * ax, (HallFloor + top) / 2f, z), new Vector3(0.12f, AviaryHeight, 0.12f), steel);
        Box("AviaryBeam_W", aviary, new Vector3(-ax, top, 0f), new Vector3(0.12f, 0.12f, 2f * az), steel);
        Box("AviaryBeam_E", aviary, new Vector3(ax, top, 0f), new Vector3(0.12f, 0.12f, 2f * az), steel);
        Box("AviaryBeam_S", aviary, new Vector3(0f, top, -az), new Vector3(2f * ax, 0.12f, 0.12f), steel);
        Box("AviaryBeam_N", aviary, new Vector3(0f, top, az), new Vector3(2f * ax, 0.12f, 0.12f), steel);
        Box("AviaryNet_W", aviary, new Vector3(-ax, (HallFloor + top) / 2f + 0.6f, 0f), new Vector3(0.01f, AviaryHeight - 1.2f, 2f * az), net);
        Box("AviaryNet_E", aviary, new Vector3(ax, (HallFloor + top) / 2f + 0.6f, 0f), new Vector3(0.01f, AviaryHeight - 1.2f, 2f * az), net);
        Box("AviaryNet_S", aviary, new Vector3(0f, (HallFloor + top) / 2f + 0.6f, -az), new Vector3(2f * ax, AviaryHeight - 1.2f, 0.01f), net);
        Box("AviaryNet_N", aviary, new Vector3(0f, (HallFloor + top) / 2f + 0.6f, az), new Vector3(2f * ax, AviaryHeight - 1.2f, 0.01f), net);
        Box("AviaryNet_Top", aviary, new Vector3(0f, top, 0f), new Vector3(2f * ax, 0.01f, 2f * az), net);

        // workstation in a corner (table, two screens, chair)
        Transform desk = Child(p, "Workstation");
        Material wood = Mat("DeskTop", new Color(0.82f, 0.78f, 0.7f), 0.3f);
        Material screen = Mat("Screen", new Color(0.05f, 0.06f, 0.08f), 0.8f);
        Box("Table", desk, new Vector3(-8.8f, HallFloor + 0.75f, 14.5f), new Vector3(1.6f, 0.05f, 3.2f), wood);
        foreach (float dz in new[] { -1.4f, 1.4f }) foreach (float dx in new[] { -0.7f, 0.7f })
            Box("TableLeg", desk, new Vector3(-8.8f + dx, HallFloor + 0.37f, 14.5f + dz), new Vector3(0.05f, 0.74f, 0.05f), steel);
        Box("Screen_1", desk, new Vector3(-9.35f, HallFloor + 1.05f, 14.0f), new Vector3(0.04f, 0.38f, 0.62f), screen);
        Box("Screen_2", desk, new Vector3(-9.35f, HallFloor + 1.05f, 14.8f), new Vector3(0.04f, 0.38f, 0.62f), screen);
        Box("Chair", desk, new Vector3(-8.0f, HallFloor + 0.45f, 14.4f), new Vector3(0.5f, 0.08f, 0.5f), screen);

        // lights under the roof
        Transform lights = Child(p, "CeilingLights");
        Material lamp = Emissive("CeilingLamp", new Color(1f, 0.97f, 0.9f), 2.5f);
        for (int i = 0; i < 3; i++)
            foreach (float sx in new[] { -1f, 1f })
            {
                Vector3 pos = new Vector3(sx * 5.5f, Eave - 0.4f, -12f + i * 12f);
                Box("Lamp", lights, pos, new Vector3(1.2f, 0.12f, 0.5f), lamp);
                var l = new GameObject("PointLight").AddComponent<Light>();
                l.transform.SetParent(lights, false);
                l.transform.localPosition = pos - Vector3.up * 0.3f;
                l.type = LightType.Point;
                l.range = 22f;
                l.intensity = 6f;
                l.color = new Color(1f, 0.96f, 0.88f);
                l.shadows = LightShadows.None;
            }
    }

    // ================================================================== ramp, signs
    static void BuildRamp(Transform p)
    {
        Material concrete = Mat("Concrete", new Color(0.62f, 0.63f, 0.64f), 0.15f);
        Material steel = Mat("Steel", new Color(0.70f, 0.72f, 0.75f), 0.6f, 0.8f);
        float x0 = HallWidth / 2f + 0.5f, w = 3f, zStart = -9f, zTop = 6f, zEnd = DoorZ + DoorWidth / 2f + 1f;
        // ramp body (side profile extruded across its width): rises from the street to the hall floor, then a landing
        var body = new[] { new Vector3(x0, Ground, zStart), new Vector3(x0, HallFloor, zTop), new Vector3(x0, HallFloor, zEnd), new Vector3(x0, Ground, zEnd) };
        MeshObject("RampBody", p, Prism(body, new Vector3(w, 0f, 0f)), concrete);
        // parapet walls on both sides (1 m above the ramp surface)
        foreach (float x in new[] { x0 - 0.2f, x0 + w })
        {
            var wall = new[] { new Vector3(x, Ground, zStart), new Vector3(x, Ground + 1f, zStart), new Vector3(x, HallFloor + 1f, zTop), new Vector3(x, HallFloor + 1f, zEnd), new Vector3(x, Ground, zEnd) };
            MeshObject("Parapet", p, Prism(wall, new Vector3(0.2f, 0f, 0f)), concrete);
        }
        // handrail along the outer parapet
        Vector3 a = new Vector3(x0 + w + 0.1f, Ground + 1.05f, zStart), b = new Vector3(x0 + w + 0.1f, HallFloor + 1.05f, zTop);
        var rail = Box("Handrail", p, (a + b) / 2f, new Vector3(0.05f, 0.05f, (b - a).magnitude), steel);
        rail.transform.rotation = Quaternion.LookRotation(b - a);
        Box("Handrail_Landing", p, new Vector3(x0 + w + 0.1f, HallFloor + 1.05f, (zTop + zEnd) / 2f), new Vector3(0.05f, 0.05f, zEnd - zTop), steel);

        // road signs at the foot of the ramp (no entry, restricted access, 3.5 t)
        Transform sign = Child(p, "RoadSigns");
        Vector3 basePos = new Vector3(x0 + w + 1.2f, Ground, zStart - 1.5f);
        Box("SignPost", sign, basePos + Vector3.up * 1.4f, new Vector3(0.07f, 2.8f, 0.07f), steel);
        Decal("NoEntry", sign, "sign_no_entry", basePos + new Vector3(0f, 2.45f, -0.05f), Vector3.back, 0.7f);
        Decal("Restricted", sign, "sign_restricted", basePos + new Vector3(0f, 1.95f, -0.05f), Vector3.back, 0.55f);
        Decal("WeightLimit", sign, "sign_weight_limit", basePos + new Vector3(0f, 1.45f, -0.05f), Vector3.back, 0.7f);
    }

    // ================================================================== surroundings
    static void BuildNeighbours(Transform p)
    {
        Material metal = Mat("GreyCladding", new Color(0.62f, 0.65f, 0.68f), 0.45f, 0.3f);
        Material roof = Mat("GreyRoof", new Color(0.55f, 0.57f, 0.6f), 0.5f, 0.4f);
        Material window = Mat("Window", new Color(0.25f, 0.32f, 0.38f), 0.85f);
        Building("Workshop_1", p, new Vector3(4f, 0f, 36f), new Vector3(28f, 4.5f, 12f), metal, roof, window);
        Building("Workshop_2", p, new Vector3(-48f, 0f, 48f), new Vector3(22f, 5f, 12f), metal, roof, window);
        Building("Workshop_3", p, new Vector3(36f, 0f, 30f), new Vector3(14f, 4f, 18f), metal, roof, window);
    }

    static void Building(string name, Transform p, Vector3 pos, Vector3 size, Material walls, Material roof, Material window)
    {
        Transform b = Child(p, name);
        b.localPosition = new Vector3(pos.x, Ground, pos.z);
        Box("Walls", b, new Vector3(0f, size.y / 2f, 0f), size, walls).transform.localPosition = new Vector3(0f, size.y / 2f, 0f);
        var r = MeshObject("Roof", b, Prism(new[] { new Vector3(-size.x / 2f - 0.3f, size.y, -size.z / 2f - 0.3f), new Vector3(0f, size.y + 1.4f, -size.z / 2f - 0.3f),
                                                   new Vector3(size.x / 2f + 0.3f, size.y, -size.z / 2f - 0.3f) }, new Vector3(0f, 0f, size.z + 0.6f)), roof);
        r.transform.localPosition = Vector3.zero;
        for (float x = -size.x / 2f + 2f; x < size.x / 2f - 1f; x += 4f)
        {
            var w = Box("Window", b, Vector3.zero, new Vector3(1.6f, 1.1f, 0.05f), window);
            w.transform.localPosition = new Vector3(x, size.y * 0.55f, -size.z / 2f - 0.03f);
        }
    }

    static void BuildFence(Transform p)
    {
        Material blue = Mat("FenceBlue", new Color(0.22f, 0.48f, 0.72f), 0.45f, 0.5f);
        float x = 30f;
        Box("FenceRail_Top", p, new Vector3(x, Ground + 1.9f, 0f), new Vector3(0.06f, 0.06f, 90f), blue);
        Box("FenceRail_Low", p, new Vector3(x, Ground + 0.15f, 0f), new Vector3(0.06f, 0.06f, 90f), blue);
        for (float z = -45f; z <= 45f; z += 0.25f)
            Box("FenceBar", p, new Vector3(x, Ground + 1f, z), new Vector3(0.03f, 2f, 0.03f), blue);
    }

    static void BuildTrees(Transform p)
    {
        Material trunk = Mat("TreeTrunk", new Color(0.33f, 0.24f, 0.16f), 0.1f);
        Material[] leaves =
        {
            Mat("Leaves_Green", new Color(0.24f, 0.42f, 0.18f), 0.1f),
            Mat("Leaves_DarkGreen", new Color(0.17f, 0.32f, 0.15f), 0.1f),
            Mat("Leaves_Autumn", new Color(0.62f, 0.45f, 0.18f), 0.1f),
            Mat("Leaves_Red", new Color(0.55f, 0.27f, 0.15f), 0.1f),
        };
        var spots = new List<Vector3>();
        for (float z = -50f; z <= 50f; z += 11f) spots.Add(new Vector3(-27f, 0f, z + 3f));        // strip between road and parking
        for (float z = -40f; z <= 40f; z += 16f) spots.Add(new Vector3(-62f, 0f, z));              // behind the parking lot
        for (float z = -35f; z <= 35f; z += 12f) spots.Add(new Vector3(36f, 0f, z));               // behind the fence
        spots.Add(new Vector3(26f, 0f, -20f)); spots.Add(new Vector3(25f, 0f, 14f));
        spots.Add(new Vector3(-6f, 0f, -42f)); spots.Add(new Vector3(8f, 0f, -44f)); spots.Add(new Vector3(-30f, 0f, 58f));
        for (int k = 0; k < 90; k++)                                                              // distant tree line (hides the horizon)
        {
            float a = k * 2f * Mathf.PI / 90f, d = R(115f, 160f);
            spots.Add(new Vector3(Mathf.Cos(a) * d, 0f, Mathf.Sin(a) * d));
        }
        int i = 0;
        foreach (Vector3 s in spots)
        {
            Transform t = Child(p, "Tree_" + i++);
            t.localPosition = new Vector3(s.x + R(-1f, 1f), Ground, s.z + R(-1f, 1f));
            bool far = s.magnitude > 100f;
            float h = R(3f, 5f) * (far ? 1.6f : 1f), scale = R(0.8f, 1.3f) * (far ? 1.8f : 1f);
            Prim(PrimitiveType.Cylinder, "Trunk", t, new Vector3(0f, h / 2f, 0f), Vector3.zero, new Vector3(0.35f, h / 2f, 0.35f), trunk);
            Material leaf = leaves[rng.Next(leaves.Length)];
            for (int k = 0; k < 4; k++)
            {
                float r = R(2.2f, 3.2f) * scale;
                Prim(PrimitiveType.Sphere, "Canopy", t, new Vector3(R(-1.2f, 1.2f), h + R(0.3f, 2f) * scale, R(-1.2f, 1.2f)), Vector3.zero, new Vector3(r, r * 0.85f, r), leaf);
            }
        }
    }

    static void BuildCars(Transform p)
    {
        Color[] paints = { new Color(0.92f, 0.92f, 0.93f), new Color(0.75f, 0.77f, 0.79f), new Color(0.12f, 0.12f, 0.13f),
                           new Color(0.35f, 0.37f, 0.4f), new Color(0.15f, 0.22f, 0.4f), new Color(0.55f, 0.08f, 0.08f) };
        int n = 0;
        foreach (float x in new[] { -33f, -40.5f, -45.5f, -53f })   // facing the stall lines of the three rows
            for (int i = 0; i < 24; i++)
            {
                if (rng.NextDouble() < 0.45) continue;                 // free stalls
                float z = -31.7f + i * 2.6f;
                bool van = rng.NextDouble() < 0.12;
                Car("Car_" + n++, p, new Vector3(x, Ground, z), x < -43f ? 90f : -90f, paints[rng.Next(paints.Length)], van);
            }
        Car("Car_" + n++, p, new Vector3(-16.5f, Ground, 20f), 0f, paints[0], true);                 // parked along the street
        Car("Car_" + n++, p, new Vector3(18f, Ground, -12f), 180f, paints[2], false);                // service yard
    }

    // Low-poly car: body, glass cabin, four wheels (along local Z)
    static void Car(string name, Transform p, Vector3 pos, float heading, Color paint, bool van)
    {
        Transform c = Child(p, name);
        c.localPosition = pos;
        c.localRotation = Quaternion.Euler(0f, heading, 0f);
        Material body = Mat("CarPaint_" + ColorUtility.ToHtmlStringRGB(paint), paint, 0.75f, 0.3f);
        Material glass = Mat("CarGlass", new Color(0.12f, 0.15f, 0.18f), 0.9f);
        Material tyre = Mat("Tyre", new Color(0.06f, 0.06f, 0.06f), 0.2f);
        float length = van ? 5f : 4.3f, height = van ? 1.45f : 0.75f;
        Box("Body", c, Vector3.zero, new Vector3(1.8f, height, length), body).transform.localPosition = new Vector3(0f, 0.35f + height / 2f, 0f);
        Box("Cabin", c, Vector3.zero, new Vector3(1.6f, van ? 0.55f : 0.6f, van ? 3.4f : 2.3f), glass).transform.localPosition = new Vector3(0f, 0.35f + height + 0.28f, van ? 0.6f : -0.2f);
        if (!van) Box("Roof", c, Vector3.zero, new Vector3(1.58f, 0.06f, 2.0f), body).transform.localPosition = new Vector3(0f, 0.35f + height + 0.6f, -0.25f);
        foreach (float sx in new[] { -0.82f, 0.82f })
            foreach (float sz in new[] { -length * 0.33f, length * 0.33f })
                Prim(PrimitiveType.Cylinder, "Wheel", c, new Vector3(sx, 0.33f, sz), new Vector3(0f, 0f, 90f), new Vector3(0.66f, 0.12f, 0.66f), tyre);
    }

    static void BuildStreetFurniture(Transform p)
    {
        Material pole = Mat("LampPole", new Color(0.55f, 0.57f, 0.6f), 0.5f, 0.6f);
        Material head = Emissive("StreetLampHead", new Color(1f, 0.95f, 0.85f), 0.3f);
        Material cabinet = Mat("Cabinet", new Color(0.86f, 0.83f, 0.74f), 0.3f);
        for (float z = -50f; z <= 50f; z += 20f)
        {
            Transform l = Child(p, "StreetLamp");
            l.localPosition = new Vector3(-15.5f, Ground, z);
            Prim(PrimitiveType.Cylinder, "Pole", l, new Vector3(0f, 4f, 0f), Vector3.zero, new Vector3(0.14f, 4f, 0.14f), pole);
            Box("Head", l, Vector3.zero, new Vector3(0.9f, 0.15f, 0.35f), head).transform.localPosition = new Vector3(-0.35f, 8f, 0f);
        }
        Box("Cabinet_1", p, new Vector3(-12f, Ground + 0.6f, -24f), new Vector3(0.6f, 1.2f, 1.3f), cabinet);
        Box("Cabinet_2", p, new Vector3(-12f, Ground + 0.6f, -22f), new Vector3(0.6f, 1.2f, 1.3f), cabinet);
        Box("Cabinet_3", p, new Vector3(16f, Ground + 0.5f, -24f), new Vector3(1.2f, 1.0f, 0.6f), cabinet);
    }

    // ================================================================== lighting, camera, robot
    static void SetupLighting()
    {
        Light sun = Object.FindAnyObjectByType<Light>();
        foreach (var l in Object.FindObjectsByType<Light>(FindObjectsSortMode.None)) if (l.type == LightType.Directional) sun = l;
        sun.name = "Sun";
        sun.transform.rotation = Quaternion.Euler(38f, -40f, 0f);
        sun.intensity = 1.5f;
        sun.color = new Color(1f, 0.96f, 0.9f);
        sun.shadows = LightShadows.Soft;

        var sky = AssetDatabase.LoadAssetAtPath<Material>(MaterialFolder + "/Sky.mat");
        if (sky == null) { sky = new Material(Shader.Find("Skybox/Procedural")); AssetDatabase.CreateAsset(sky, MaterialFolder + "/Sky.mat"); }
        sky.SetFloat("_SunSize", 0.035f);
        sky.SetFloat("_AtmosphereThickness", 0.9f);
        sky.SetColor("_SkyTint", new Color(0.45f, 0.6f, 0.85f));
        sky.SetColor("_GroundColor", new Color(0.33f, 0.42f, 0.28f));
        sky.SetFloat("_Exposure", 1.25f);
        RenderSettings.skybox = sky;
        RenderSettings.sun = sun;
        RenderSettings.ambientMode = AmbientMode.Skybox;
        RenderSettings.ambientIntensity = 1.1f;
        RenderSettings.fog = false;                                   // the UnderwaterEffect turns fog on under water only

        // post-processing: filmic tonemapping, a little bloom, contrast and saturation, light vignette
        string profilePath = "Assets/Environment/Settings/CampusPostProcessing.asset";
        if (!AssetDatabase.IsValidFolder("Assets/Environment/Settings")) AssetDatabase.CreateFolder("Assets/Environment", "Settings");
        AssetDatabase.DeleteAsset(profilePath);
        var profile = ScriptableObject.CreateInstance<VolumeProfile>();
        AssetDatabase.CreateAsset(profile, profilePath);
        var tone = profile.Add<Tonemapping>(true); tone.mode.Override(TonemappingMode.ACES);
        var bloom = profile.Add<Bloom>(true); bloom.intensity.Override(0.25f); bloom.threshold.Override(1.1f);
        var color = profile.Add<ColorAdjustments>(true); color.postExposure.Override(0.25f); color.contrast.Override(10f); color.saturation.Override(8f);
        var vignette = profile.Add<Vignette>(true); vignette.intensity.Override(0.18f);
        foreach (var c in profile.components) AssetDatabase.AddObjectToAsset(c, profile);
        EditorUtility.SetDirty(profile);
        var volume = new GameObject("PostProcessing").AddComponent<Volume>();
        volume.isGlobal = true;
        volume.sharedProfile = profile;
    }

    static void SetupCamera()
    {
        Camera cam = Camera.main;
        cam.farClipPlane = 600f;
        cam.nearClipPlane = 0.05f;
        var data = cam.GetUniversalAdditionalCameraData();
        data.renderPostProcessing = true;
        data.antialiasing = AntialiasingMode.SubpixelMorphologicalAntiAliasing;
        var tour = cam.GetComponent<CameraTour>();
        if (tour == null) tour = cam.gameObject.AddComponent<CameraTour>();
        tour.waypoints = new List<CameraTour.Waypoint>
        {
            WP(new Vector3(-80f, 42f, -75f), new Vector3(0f, 8f, 0f), 6f),          // aerial view from the parking lot
            WP(new Vector3(-42f, 14f, -26f), new Vector3(-11f, 10f, 4f), 6f),        // west facade, logos
            WP(new Vector3(-6f, 16f, -52f), new Vector3(4f, 9f, -8f), 6f),           // south end, arched wall and domes
            WP(new Vector3(30f, 9f, -30f), new Vector3(13f, 6f, -2f), 5f),          // entrance side, ramp and sign
            WP(new Vector3(13f, 5.2f, -10f), new Vector3(13f, 7.5f, 8f), 5f),       // climbing the ramp
            WP(new Vector3(13.5f, 7.8f, 10f), new Vector3(10f, 7.6f, 11f), 4f),     // at the door
            WP(new Vector3(8.5f, 8.2f, 11f), new Vector3(0f, 6.5f, 4f), 4f),        // entering the hall
            WP(new Vector3(4f, 10.5f, 4f), new Vector3(0f, 5f, -4f), 5f),           // over the basin, inside the aviary
            WP(new Vector3(0f, 3.9f, -8.6f), new Vector3(0f, 3f, -6f), 1f),         // behind the robot (follow camera)
        };
        tour.playOnStart = true;
    }

    static CameraTour.Waypoint WP(Vector3 p, Vector3 look, float d) => new CameraTour.Waypoint { position = p, lookAt = look, duration = d };

    // In this presentation scene the robot is neutral (moving mass 2.28 kg), so it stays submerged and can be driven
    static void SetupRobot()
    {
        var physics = Object.FindAnyObjectByType<RobotFishPhysics>();
        physics.movingMass = 2.282f;
        EditorUtility.SetDirty(physics);
        var log = physics.GetComponent<GroundTruthLogger>();
        if (log != null) Object.DestroyImmediate(log);                // no ground-truth files from presentations
    }

    static void AddToBuildSettings()
    {
        var scenes = new List<EditorBuildSettingsScene>(EditorBuildSettings.scenes);
        if (!scenes.Exists(s => s.path == ScenePath)) scenes.Add(new EditorBuildSettingsScene(ScenePath, true));
        EditorBuildSettings.scenes = scenes.ToArray();
    }

    // ================================================================== helpers
    static float R(float a, float b) => a + (float)rng.NextDouble() * (b - a);

    static Transform Child(Transform parent, string name)
    {
        var t = new GameObject(name).transform;
        t.SetParent(parent, false);
        return t;
    }

    static GameObject Box(string name, Transform parent, Vector3 pos, Vector3 size, Material mat) =>
        Prim(PrimitiveType.Cube, name, parent, pos, Vector3.zero, size, mat);

    static GameObject Prim(PrimitiveType type, string name, Transform parent, Vector3 pos, Vector3 euler, Vector3 scale, Material mat)
    {
        GameObject go = GameObject.CreatePrimitive(type);
        go.name = name;
        if (type != PrimitiveType.Cube || scale.y < 0.5f) Object.DestroyImmediate(go.GetComponent<Collider>());  // keep colliders on walls only
        go.transform.SetParent(parent, false);
        go.transform.localPosition = pos;
        go.transform.localEulerAngles = euler;
        go.transform.localScale = scale;
        go.GetComponent<MeshRenderer>().sharedMaterial = mat;
        return go;
    }

    // Wall along Z at x, between y0 and y1, from z0 to z1, with an opening from a to b
    static void SegmentZ(string name, Transform p, float x, float y0, float y1, float z0, float a, float b, float z1, Material mat)
    {
        Box(name + "_1", p, new Vector3(x, (y0 + y1) / 2f, (z0 + a) / 2f), new Vector3(WallThickness, y1 - y0, a - z0), mat);
        Box(name + "_2", p, new Vector3(x, (y0 + y1) / 2f, (b + z1) / 2f), new Vector3(WallThickness, y1 - y0, z1 - b), mat);
    }

    static GameObject MeshObject(string name, Transform parent, Mesh mesh, Material mat)
    {
        var go = new GameObject(name);
        go.transform.SetParent(parent, false);
        go.AddComponent<MeshFilter>().sharedMesh = mesh;
        go.AddComponent<MeshRenderer>().sharedMaterial = mat;
        return go;
    }

    // Convex polygon (world points) extruded along a vector; flat faces, normals pointing outward. The mesh is
    // saved as an asset so that the scene keeps it.
    static Mesh Prism(Vector3[] face, Vector3 extrude)
    {
        var v = new List<Vector3>();
        var tri = new List<int>();
        Vector3 center = Vector3.zero;
        foreach (var q in face) center += q;
        center = center / face.Length + extrude / 2f;
        void Quad(Vector3 a, Vector3 b, Vector3 c, Vector3 d) { Tri(a, b, c); Tri(a, c, d); }
        void Tri(Vector3 a, Vector3 b, Vector3 c)
        {
            Vector3 n = Vector3.Cross(b - a, c - a);
            if (Vector3.Dot(n, (a + b + c) / 3f - center) < 0f) { var s = b; b = c; c = s; }   // outward
            int k = v.Count;
            v.Add(a); v.Add(b); v.Add(c);
            tri.Add(k); tri.Add(k + 1); tri.Add(k + 2);
        }
        for (int i = 1; i < face.Length - 1; i++)
        {
            Tri(face[0], face[i], face[i + 1]);
            Tri(face[0] + extrude, face[i] + extrude, face[i + 1] + extrude);
        }
        for (int i = 0; i < face.Length; i++)
        {
            Vector3 a = face[i], b = face[(i + 1) % face.Length];
            Quad(a, b, b + extrude, a + extrude);
        }
        var mesh = new Mesh { name = "Prism" };
        mesh.SetVertices(v);
        mesh.SetTriangles(tri, 0);
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
        return SaveMesh(mesh);
    }

    // Unit upper hemisphere (radius 1, base on y = 0), smooth normals
    static Mesh Hemisphere(int segments, int rings)
    {
        var v = new List<Vector3>();
        var tri = new List<int>();
        for (int r = 0; r <= rings; r++)
        {
            float phi = Mathf.PI / 2f * r / rings;                    // 0 = base, π/2 = top
            for (int s = 0; s <= segments; s++)
            {
                float th = 2f * Mathf.PI * s / segments;
                v.Add(new Vector3(Mathf.Cos(phi) * Mathf.Cos(th), Mathf.Sin(phi), Mathf.Cos(phi) * Mathf.Sin(th)));
            }
        }
        for (int r = 0; r < rings; r++)
            for (int s = 0; s < segments; s++)
            {
                int a = r * (segments + 1) + s, b = a + segments + 1;
                tri.AddRange(new[] { a, b, a + 1, a + 1, b, b + 1 });
            }
        var mesh = new Mesh { name = "Hemisphere" };
        mesh.SetVertices(v);
        mesh.SetTriangles(tri, 0);
        mesh.RecalculateNormals();
        // make sure the faces point outward
        if (Vector3.Dot(mesh.normals[segments + 2], v[segments + 2]) < 0f) { tri.Reverse(); mesh.SetTriangles(tri, 0); mesh.RecalculateNormals(); }
        mesh.RecalculateBounds();
        return SaveMesh(mesh);
    }

    static int meshCount;
    static Mesh SaveMesh(Mesh mesh)
    {
        string folder = "Assets/Environment/Meshes";
        if (!AssetDatabase.IsValidFolder(folder)) AssetDatabase.CreateFolder("Assets/Environment", "Meshes");
        if (meshCount == 0) foreach (string guid in AssetDatabase.FindAssets("t:Mesh", new[] { folder })) AssetDatabase.DeleteAsset(AssetDatabase.GUIDToAssetPath(guid));
        AssetDatabase.CreateAsset(mesh, $"{folder}/{mesh.name}_{meshCount++:D2}.asset");
        return mesh;
    }

    // Image on a wall: a quad with the texture (alpha-clipped), facing `normal`, `width` metres wide
    static void Decal(string name, Transform parent, string texture, Vector3 pos, Vector3 normal, float width, float rollDeg = 0f)
    {
        var tex = AssetDatabase.LoadAssetAtPath<Texture2D>($"{TextureFolder}/{texture}.png");
        Material mat = Mat("Decal_" + texture, Color.white, 0.3f);
        mat.SetTexture("_BaseMap", tex);
        mat.SetFloat("_AlphaClip", 1f);
        mat.SetFloat("_Cutoff", 0.4f);
        mat.EnableKeyword("_ALPHATEST_ON");
        float aspect = (float)tex.height / tex.width;
        var go = Prim(PrimitiveType.Quad, name, parent, pos, Vector3.zero, new Vector3(width, width * aspect, 1f), mat);
        go.transform.rotation = Quaternion.LookRotation(-normal, Vector3.up) * Quaternion.Euler(0f, 0f, rollDeg);   // a quad faces −Z
    }

    static Material Mat(string name, Color color, float smoothness, float metallic = 0f)
    {
        if (materials.TryGetValue(name, out Material cached)) return cached;
        if (!AssetDatabase.IsValidFolder(MaterialFolder)) AssetDatabase.CreateFolder("Assets/Environment", "Materials");
        string path = $"{MaterialFolder}/{name}.mat";
        var mat = AssetDatabase.LoadAssetAtPath<Material>(path);
        if (mat == null) { mat = new Material(Shader.Find("Universal Render Pipeline/Lit")); AssetDatabase.CreateAsset(mat, path); }
        mat.SetColor("_BaseColor", color);
        mat.SetFloat("_Smoothness", smoothness);
        mat.SetFloat("_Metallic", metallic);
        EditorUtility.SetDirty(mat);
        materials[name] = mat;
        return mat;
    }

    static Material Transparent(string name, Color color, float smoothness, bool doubleSided = false)
    {
        Material mat = Mat(name, color, smoothness);
        mat.SetFloat("_Surface", 1f);
        mat.SetFloat("_Blend", 0f);
        mat.SetFloat("_SrcBlend", (float)BlendMode.SrcAlpha);
        mat.SetFloat("_DstBlend", (float)BlendMode.OneMinusSrcAlpha);
        mat.SetFloat("_ZWrite", 0f);
        mat.EnableKeyword("_SURFACE_TYPE_TRANSPARENT");
        mat.renderQueue = (int)RenderQueue.Transparent;
        mat.SetOverrideTag("RenderType", "Transparent");
        if (doubleSided) mat.SetFloat("_Cull", (float)CullMode.Off);
        return mat;
    }

    static Material Emissive(string name, Color color, float intensity)
    {
        Material mat = Mat(name, color, 0.5f);
        mat.EnableKeyword("_EMISSION");
        mat.SetColor("_EmissionColor", color * intensity);
        mat.globalIlluminationFlags = MaterialGlobalIlluminationFlags.RealtimeEmissive;
        return mat;
    }

    // Logo textures: keep their transparency, no repetition, full resolution
    static void ConfigureTextures()
    {
        foreach (string guid in AssetDatabase.FindAssets("t:Texture2D", new[] { TextureFolder }))
        {
            var importer = (TextureImporter)AssetImporter.GetAtPath(AssetDatabase.GUIDToAssetPath(guid));
            importer.alphaIsTransparency = true;
            importer.wrapMode = TextureWrapMode.Clamp;
            importer.maxTextureSize = 4096;
            importer.mipmapEnabled = true;
            importer.SaveAndReimport();
        }
    }
}
