using UnityEngine;                                    // core Unity API
using UnityEngine.Rendering;                          // AsyncGPUReadback
using Unity.Robotics.ROSTCPConnector;                 // ROSConnection
using RosMessageTypes.Sensor;                         // ImageMsg, CompressedImageMsg

// RGB camera of the AUV. The camera renders into a RenderTexture; at a fixed rate the image is copied
// back from the GPU asynchronously and published as JPEG (sensor_msgs/CompressedImage) and/or raw rgb8 (sensor_msgs/Image).
[RequireComponent(typeof(Camera))]                    // must sit on a Camera
public class CameraPublisher : MonoBehaviour
{
    [Header("Image")]                                 // Inspector section title
    public int width = 640;                           // image width [px]
    public int height = 480;                          // image height [px]
    public float publishRate = 10f;                   // images per second [Hz]
    [Range(10, 100)] public int jpegQuality = 75;     // JPEG quality (higher = better image, bigger message)

    [Header("ROS 2")]                                 // Inspector section title
    public bool publishCompressed = true;             // publish JPEG (light, recommended)
    public string compressedTopic = "camera/compressed_image"; // CompressedImage topic
    public bool publishRaw = false;                   // publish raw rgb8 (heavy: ~9 MB/s at 640×480, 10 Hz)
    public string rawTopic = "camera/image";          // Image topic
    public string frameId = "camera_link";            // frame name in the message header

    [Header("Preview")]                               // Inspector section title
    public bool showPreview = true;                   // picture-in-picture of the robot camera in the Game view
    public float previewWidth = 320f;                 // preview width on screen [px]

    [Header("Debug (read-only)")]                     // Inspector section title
    public int imagesPublished;                       // number of images published

    private Camera cam;                               // this camera
    private RenderTexture renderTexture;              // render target
    private Texture2D jpegTexture;                    // CPU texture used to encode JPEG
    private ROSConnection ros;                        // ROS connection
    private float nextPublishTime;                    // time of the next image
    private bool readbackPending;                     // waiting for the GPU copy?

    // Creates the render target and registers the publishers
    void Start()
    {
        cam = GetComponent<Camera>();                                                // this camera
        renderTexture = new RenderTexture(width, height, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB); // sRGB: same colors as on screen
        renderTexture.Create();                                                      // allocate on the GPU
        cam.targetTexture = renderTexture;                                           // the camera no longer draws on the screen
        jpegTexture = new Texture2D(width, height, TextureFormat.RGBA32, false);     // CPU copy for JPEG encoding

        ros = ROSConnection.GetOrCreateInstance();                                   // shared ROS connection
        if (publishCompressed) ros.RegisterPublisher<CompressedImageMsg>(compressedTopic); // JPEG topic
        if (publishRaw) ros.RegisterPublisher<ImageMsg>(rawTopic);                   // raw topic
    }

    // Requests a GPU copy at the configured rate
    void Update()
    {
        if (readbackPending || Time.time < nextPublishTime) return;                 // busy or too early
        nextPublishTime = Time.time + 1f / Mathf.Max(publishRate, 0.1f);             // schedule the next image
        readbackPending = true;                                                      // one request at a time
        AsyncGPUReadback.Request(renderTexture, 0, TextureFormat.RGBA32, OnReadback); // copy without stalling the game (RGBA works on every graphics API)
    }

    // Runs a few frames later with the image bytes (RGB, bottom row first)
    void OnReadback(AsyncGPUReadbackRequest request)
    {
        readbackPending = false;                                                     // ready for the next request
        if (request.hasError || ros == null) return;                                 // copy failed or not started
        var pixels = request.GetData<byte>();                                        // RGBA32, Unity row order (bottom first)

        if (publishCompressed)                                                       // JPEG
        {
            jpegTexture.LoadRawTextureData(pixels);                                  // fill the CPU texture
            jpegTexture.Apply(false);                                                // commit (no mipmaps)
            byte[] jpeg = jpegTexture.EncodeToJPG(jpegQuality);                      // encode; Unity writes the top row first
            ros.Publish(compressedTopic, new CompressedImageMsg(SensorUtils.Header(frameId), "jpeg", jpeg)); // publish
        }

        if (publishRaw)                                                              // raw rgb8
        {
            int rowBytes = width * 3;                                                // bytes per row
            var data = new byte[rowBytes * height];                                  // ROS image buffer
            for (int y = 0; y < height; y++)                                         // flip vertically: ROS starts with the top row
            {
                int src = (height - 1 - y) * width * 4;                              // first byte of the source row (RGBA)
                int dst = y * rowBytes;                                              // first byte of the destination row (RGB)
                for (int x = 0; x < width; x++, src += 4)                            // each pixel
                {
                    data[dst++] = pixels[src];                                       // R
                    data[dst++] = pixels[src + 1];                                   // G
                    data[dst++] = pixels[src + 2];                                   // B (alpha dropped)
                }
            }
            ros.Publish(rawTopic, new ImageMsg(SensorUtils.Header(frameId), (uint)height, (uint)width, "rgb8", 0, (uint)rowBytes, data)); // publish
        }
        imagesPublished++;                                                           // count
    }

    // Draws the robot camera image in the bottom-right corner of the Game view
    void OnGUI()
    {
        if (!showPreview || renderTexture == null) return;                           // preview off or not ready
        float h = previewWidth * height / width;                                     // keep the aspect ratio
        var rect = new Rect(Screen.width - previewWidth - 12f, Screen.height - h - 12f, previewWidth, h); // bottom-right corner
        GUI.DrawTexture(rect, renderTexture, ScaleMode.ScaleToFit, false);           // draw the image
        GUI.Label(new Rect(rect.x + 6f, rect.y + 4f, previewWidth, 20f), $"RGB camera  #{imagesPublished}"); // caption
    }

    // Releases the GPU and CPU buffers
    void OnDestroy()
    {
        if (cam != null) cam.targetTexture = null;                                   // detach the render target
        if (renderTexture != null) renderTexture.Release();                          // free GPU memory
        if (jpegTexture != null) Destroy(jpegTexture);                               // free CPU texture
    }
}
