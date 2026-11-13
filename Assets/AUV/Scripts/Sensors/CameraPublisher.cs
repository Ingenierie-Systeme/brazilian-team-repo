using UnityEngine;
using UnityEngine.Rendering;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

// RGB camera of the AUV. The camera renders into a RenderTexture; at a fixed rate the image is copied
// back from the GPU asynchronously and published as JPEG (sensor_msgs/CompressedImage) and/or raw rgb8 (sensor_msgs/Image).
[RequireComponent(typeof(Camera))]
public class CameraPublisher : MonoBehaviour
{
    [Header("Image")]
    public int width = 640;                           // image width [px]
    public int height = 480;                          // image height [px]
    public float publishRate = 10f;                   // images per second [Hz]
    [Range(10, 100)] public int jpegQuality = 75;     // JPEG quality (higher = better image, bigger message)

    [Header("ROS 2")]
    public bool publishCompressed = true;             // publish JPEG (light, recommended)
    public string compressedTopic = "camera/compressed_image"; // CompressedImage topic
    public bool publishRaw = false;                   // publish raw rgb8 (heavy: ~9 MB/s at 640×480, 10 Hz)
    public string rawTopic = "camera/image";          // Image topic
    public string frameId = "camera_link";            // frame name in the message header

    [Header("Preview")]
    public bool showPreview = true;                   // picture-in-picture of the robot camera in the Game view
    public float previewWidth = 320f;                 // preview width on screen [px]

    [Header("Debug (read-only)")]
    public int imagesPublished;                       // number of images published

    private Camera cam;
    private RenderTexture renderTexture;              // render target
    private Texture2D jpegTexture;                    // CPU texture used to encode JPEG
    private ROSConnection ros;
    private float nextPublishTime;                    // time of the next image
    private bool readbackPending;                     // waiting for the GPU copy?

    // Creates the render target and registers the publishers
    void Start()
    {
        cam = GetComponent<Camera>();
        renderTexture = new RenderTexture(width, height, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB); // sRGB: same colors as on screen
        renderTexture.Create();
        cam.targetTexture = renderTexture;                                           // the camera no longer draws on the screen
        jpegTexture = new Texture2D(width, height, TextureFormat.RGBA32, false);

        ros = ROSConnection.GetOrCreateInstance();
        if (publishCompressed) ros.RegisterPublisher<CompressedImageMsg>(compressedTopic);
        if (publishRaw) ros.RegisterPublisher<ImageMsg>(rawTopic);
    }

    // Requests a GPU copy at the configured rate
    void Update()
    {
        if (readbackPending || Time.time < nextPublishTime) return;
        nextPublishTime = Time.time + 1f / Mathf.Max(publishRate, 0.1f);
        readbackPending = true;
        AsyncGPUReadback.Request(renderTexture, 0, TextureFormat.RGBA32, OnReadback); // copy without stalling the game (RGBA works on every graphics API)
    }

    // Runs a few frames later with the image bytes (RGB, bottom row first)
    void OnReadback(AsyncGPUReadbackRequest request)
    {
        readbackPending = false;
        if (request.hasError || ros == null) return;
        var pixels = request.GetData<byte>();                                        // RGBA32, Unity row order (bottom first)

        if (publishCompressed)
        {
            jpegTexture.LoadRawTextureData(pixels);
            jpegTexture.Apply(false);
            byte[] jpeg = jpegTexture.EncodeToJPG(jpegQuality);                      // encode; Unity writes the top row first
            ros.Publish(compressedTopic, new CompressedImageMsg(SensorUtils.Header(frameId), "jpeg", jpeg));
        }

        if (publishRaw)
        {
            int rowBytes = width * 3;
            var data = new byte[rowBytes * height];
            for (int y = 0; y < height; y++)                                         // flip vertically: ROS starts with the top row
            {
                int src = (height - 1 - y) * width * 4;
                int dst = y * rowBytes;
                for (int x = 0; x < width; x++, src += 4)
                {
                    data[dst++] = pixels[src];
                    data[dst++] = pixels[src + 1];
                    data[dst++] = pixels[src + 2];                                   // B (alpha dropped)
                }
            }
            ros.Publish(rawTopic, new ImageMsg(SensorUtils.Header(frameId), (uint)height, (uint)width, "rgb8", 0, (uint)rowBytes, data));
        }
        imagesPublished++;
    }

    // Draws the robot camera image in the bottom-right corner of the Game view
    void OnGUI()
    {
        if (!showPreview || renderTexture == null) return;
        float h = previewWidth * height / width;
        var rect = new Rect(Screen.width - previewWidth - 12f, Screen.height - h - 12f, previewWidth, h);
        GUI.DrawTexture(rect, renderTexture, ScaleMode.ScaleToFit, false);
        GUI.Label(new Rect(rect.x + 6f, rect.y + 4f, previewWidth, 20f), $"RGB camera  #{imagesPublished}");
    }

    // Releases the GPU and CPU buffers
    void OnDestroy()
    {
        if (cam != null) cam.targetTexture = null;
        if (renderTexture != null) renderTexture.Release();
        if (jpegTexture != null) Destroy(jpegTexture);
    }
}
