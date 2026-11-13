using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.SceneManagement;

// Start menu of the simulator build (scene Assets/Menu/Menu.unity, the first scene of the build): it opens one of the
// simulation scenes. In a simulation scene, Esc comes back to this menu.
//
// A scene can also be opened directly from the command line, without the menu:
//     ./RobotFishSimulator.x86_64 -scene Basin_Grid
//
// The menu object stays alive across the scenes (DontDestroyOnLoad) so that Esc works everywhere. The ROS connection of
// a scene is closed before leaving it: Unity destroys its object, but its network thread would keep running.
public class SceneMenu : MonoBehaviour
{
    const string MenuScene = "Menu";

    // Scenes of the build: scene name, title and description shown in the menu
    static readonly (string scene, string title, string description)[] Scenes =
    {
        ("Basin_Grid", "Test basin with a metric grid",
         "The robot fish in the Air-Sea Test Basin (20 × 12 × 6 m), with 1 m grid lines to read distances."),
        ("Basin_NoGrid", "Plain test basin",
         "The same basin with plain walls, closer to the real one."),
        ("AirSeaTestBasin_Campus", "Campus and basin",
         "The basin building in the ENSTA campus, with a camera tour from the parking lot to the robot."),
    };

    static SceneMenu instance;

    void Awake()
    {
        if (instance != null)                                  // the menu scene was opened again: keep the first one
        {
            Destroy(gameObject);
            return;
        }
        instance = this;
        DontDestroyOnLoad(gameObject);

        string requested = SceneFromCommandLine();
        if (requested != null) SceneManager.LoadScene(requested);
    }

    // Value of "-scene <name>" if it names a scene of the menu, else null
    static string SceneFromCommandLine()
    {
        string[] args = System.Environment.GetCommandLineArgs();
        for (int i = 0; i < args.Length - 1; i++)
        {
            if (args[i] != "-scene") continue;
            foreach (var s in Scenes)
                if (s.scene == args[i + 1]) return s.scene;
            Debug.LogWarning($"Unknown scene {args[i + 1]}: showing the menu");
        }
        return null;
    }

    static bool InMenu => SceneManager.GetActiveScene().name == MenuScene;

    void Update()
    {
        if (InMenu) return;
        Keyboard keyboard = Keyboard.current;
        if (keyboard != null && keyboard.escapeKey.wasPressedThisFrame) BackToMenu();
    }

    static void BackToMenu()
    {
        ROSConnection ros = FindAnyObjectByType<ROSConnection>();
        if (ros != null) ros.Disconnect();
        SceneManager.LoadScene(MenuScene);
    }

    void OnGUI()
    {
        if (!InMenu)
        {
            GUI.Label(new Rect(10f, Screen.height - 28f, 300f, 22f), "Esc: back to the menu");
            return;
        }

        float width = Mathf.Min(620f, Screen.width - 40f);
        float x = (Screen.width - width) / 2f;
        float y = Mathf.Max(20f, Screen.height / 2f - 230f);

        var title = new GUIStyle(GUI.skin.label) { fontSize = 28, fontStyle = FontStyle.Bold, alignment = TextAnchor.MiddleCenter };
        var text = new GUIStyle(GUI.skin.label) { fontSize = 15, wordWrap = true, alignment = TextAnchor.MiddleCenter };
        var button = new GUIStyle(GUI.skin.button) { fontSize = 18, fontStyle = FontStyle.Bold };

        GUI.Label(new Rect(x, y, width, 40f), "Robot fish simulator", title);
        GUI.Label(new Rect(x, y + 42f, width, 44f),
                  "Brazilian Team, System Engineering, ENSTA 2026–2027. Choose a scene; Esc comes back here.", text);
        y += 100f;

        foreach (var s in Scenes)
        {
            if (GUI.Button(new Rect(x, y, width, 40f), s.title, button)) SceneManager.LoadScene(s.scene);
            GUI.Label(new Rect(x, y + 42f, width, 40f), s.description, text);
            y += 92f;
        }

        GUI.Label(new Rect(x, y, width, 60f),
                  "Keys: Space swim, A / D turn, W / S moving mass, C free camera. " +
                  "The ROS 2 topics need the ROS-TCP-Endpoint on this computer (see the README).", text);
        if (GUI.Button(new Rect(x + width / 2f - 60f, y + 66f, 120f, 32f), "Quit")) Application.Quit();
    }
}
