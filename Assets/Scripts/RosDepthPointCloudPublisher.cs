using System.Collections.Generic;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

[DisallowMultipleComponent]
public class RosDepthPointCloudPublisher : MonoBehaviour
{
    [SerializeField] private Camera depthCamera;
    [SerializeField] private string topicName = "/points";
    [SerializeField] private string frameId = "camera_link";
    [SerializeField] private float publishRateHz = 5f;
    [SerializeField] private int widthSamples = 160;
    [SerializeField] private int heightSamples = 120;
    [SerializeField] private int pixelStride = 2;
    [SerializeField] private float maxRange = 10f;
    [SerializeField] private LayerMask layerMask = -1;
    [SerializeField] private bool ignoreTriggerColliders = true;

    private const int PointStep = 12;
    private ROSConnection ros;
    private double nextPublishTime;
    private PointFieldMsg[] fields;

    private void Awake()
    {
        if (depthCamera == null)
        {
            depthCamera = GetComponent<Camera>();
        }

        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PointCloud2Msg>(topicName);

        fields = new PointFieldMsg[3];
        fields[0] = CreateField("x", 0);
        fields[1] = CreateField("y", 4);
        fields[2] = CreateField("z", 8);
    }

    private void FixedUpdate()
    {
        if (publishRateHz <= 0f || depthCamera == null)
        {
            return;
        }

        double now = Time.timeAsDouble;
        if (now < nextPublishTime)
        {
            return;
        }

        nextPublishTime = now + (1.0 / publishRateHz);

        List<Vector3> points = new List<Vector3>();
        QueryTriggerInteraction triggerInteraction = ignoreTriggerColliders ? QueryTriggerInteraction.Ignore : QueryTriggerInteraction.Collide;

        int stride = Mathf.Max(1, pixelStride);
        int width = Mathf.Max(1, widthSamples);
        int height = Mathf.Max(1, heightSamples);

        for (int y = 0; y < height; y += stride)
        {
            for (int x = 0; x < width; x += stride)
            {
                float u = (x + 0.5f) / width;
                float v = (y + 0.5f) / height;
                Ray ray = depthCamera.ViewportPointToRay(new Vector3(u, v, 0f));

                if (Physics.Raycast(ray, out RaycastHit hit, maxRange, layerMask, triggerInteraction))
                {
                    Vector3 localPoint = depthCamera.transform.InverseTransformPoint(hit.point);
                    Vector3 rosPoint = RosUnityConversions.UnityToRosVector(localPoint);
                    points.Add(rosPoint);
                }
            }
        }

        PointCloud2Msg cloud = new PointCloud2Msg();
        cloud.header = RosUnityConversions.CreateHeader(frameId, now);
        cloud.height = 1;
        cloud.width = (uint)points.Count;
        cloud.fields = fields;
        cloud.is_bigendian = false;
        cloud.point_step = (uint)PointStep;
        cloud.row_step = (uint)(PointStep * points.Count);
        cloud.is_dense = true;
        cloud.data = BuildPointData(points);

        ros.Publish(topicName, cloud);
    }

    private static PointFieldMsg CreateField(string name, uint offset)
    {
        PointFieldMsg field = new PointFieldMsg();
        field.name = name;
        field.offset = offset;
        field.datatype = PointFieldMsg.FLOAT32;
        field.count = 1;
        return field;
    }

    private static byte[] BuildPointData(List<Vector3> points)
    {
        byte[] data = new byte[points.Count * PointStep];
        int offset = 0;

        for (int i = 0; i < points.Count; i++)
        {
            WriteFloat(data, ref offset, points[i].x);
            WriteFloat(data, ref offset, points[i].y);
            WriteFloat(data, ref offset, points[i].z);
        }

        return data;
    }

    private static void WriteFloat(byte[] data, ref int offset, float value)
    {
        int bytes = System.BitConverter.SingleToInt32Bits(value);
        data[offset++] = (byte)(bytes & 0xFF);
        data[offset++] = (byte)((bytes >> 8) & 0xFF);
        data[offset++] = (byte)((bytes >> 16) & 0xFF);
        data[offset++] = (byte)((bytes >> 24) & 0xFF);
    }
}
