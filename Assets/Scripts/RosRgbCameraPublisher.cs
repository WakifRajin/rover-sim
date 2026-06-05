using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

[DisallowMultipleComponent]
public class RosRgbCameraPublisher : MonoBehaviour
{
    [SerializeField] private Camera rgbCamera;
    [SerializeField] private string topicName = "/camera/image_raw";
    [SerializeField] private string cameraInfoTopic = "/camera/camera_info";
    [SerializeField] private string frameId = "camera_link";
    [SerializeField] private float publishRateHz = 20f;
    [SerializeField] private bool publishCameraInfo = true;
    [SerializeField] private int renderWidth = 640;
    [SerializeField] private int renderHeight = 480;

    private ROSConnection ros;
    private double nextPublishTime;
    private RenderTexture captureTexture;
    private Texture2D captureImage;

    private void Awake()
    {
        if (rgbCamera == null)
        {
            rgbCamera = GetComponent<Camera>();
        }

        if (rgbCamera == null)
        {
            Debug.LogError("RosRgbCameraPublisher requires a Camera reference.");
            enabled = false;
            return;
        }

        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(topicName);

        if (publishCameraInfo)
        {
            ros.RegisterPublisher<CameraInfoMsg>(cameraInfoTopic);
        }

        InitializeCaptureBuffers();
    }

    private void OnDisable()
    {
        if (captureTexture != null)
        {
            captureTexture.Release();
            captureTexture = null;
        }

        if (captureImage != null)
        {
            Destroy(captureImage);
            captureImage = null;
        }
    }

    private void LateUpdate()
    {
        if (publishRateHz <= 0f || captureTexture == null || captureImage == null)
        {
            return;
        }

        double now = Time.timeAsDouble;
        if (now < nextPublishTime)
        {
            return;
        }

        nextPublishTime = now + (1.0 / publishRateHz);

        RenderTexture previousTarget = rgbCamera.targetTexture;
        RenderTexture previousActive = RenderTexture.active;

        rgbCamera.targetTexture = captureTexture;
        rgbCamera.Render();
        RenderTexture.active = captureTexture;
        captureImage.ReadPixels(new Rect(0, 0, captureTexture.width, captureTexture.height), 0, 0);
        captureImage.Apply(false);

        RenderTexture.active = previousActive;
        rgbCamera.targetTexture = previousTarget;

        HeaderMsg header = RosUnityConversions.CreateHeader(frameId, now);
        ImageMsg image = captureImage.ToImageMsg(header);
        ros.Publish(topicName, image);

        if (publishCameraInfo)
        {
            CameraInfoMsg info = CameraInfoGenerator.ConstructCameraInfoMessage(rgbCamera, header);
            ros.Publish(cameraInfoTopic, info);
        }
    }

    private void InitializeCaptureBuffers()
    {
        int width = renderWidth > 0 ? renderWidth : Mathf.Max(1, rgbCamera.pixelWidth);
        int height = renderHeight > 0 ? renderHeight : Mathf.Max(1, rgbCamera.pixelHeight);

        captureTexture = new RenderTexture(width, height, 24, RenderTextureFormat.ARGB32);
        captureTexture.Create();
        captureImage = new Texture2D(width, height, TextureFormat.RGB24, false);
    }
}
