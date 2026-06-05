using System.Collections.Generic;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

[DisallowMultipleComponent]
public class RosLidar3DPublisher : MonoBehaviour
{
    [SerializeField] private Transform lidarFrame;
    [SerializeField] private string topicName = "/lidar/points";
    [SerializeField] private string frameId = "laser_link";
    [SerializeField] private float publishRateHz = 10f;
    [SerializeField] private int horizontalSamples = 720;
    [SerializeField] private int verticalSamples = 16;
    [SerializeField] private float minHorizontalAngleDeg = -180f;
    [SerializeField] private float maxHorizontalAngleDeg = 180f;
    [SerializeField] private float minVerticalAngleDeg = -15f;
    [SerializeField] private float maxVerticalAngleDeg = 15f;
    [SerializeField] private int horizontalStride = 1;
    [SerializeField] private int verticalStride = 1;
    [SerializeField] private float minRange = 0.1f;
    [SerializeField] private float maxRange = 50f;
    [SerializeField] private LayerMask layerMask = -1;
    [SerializeField] private bool ignoreTriggerColliders = true;

    private const int PointStep = 12;
    private ROSConnection ros;
    private double nextPublishTime;
    private PointFieldMsg[] fields;

    private void Awake()
    {
        if (lidarFrame == null)
        {
            lidarFrame = transform;
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
        if (publishRateHz <= 0f || horizontalSamples <= 0 || verticalSamples <= 0)
        {
            return;
        }

        double now = Time.timeAsDouble;
        if (now < nextPublishTime)
        {
            return;
        }

        nextPublishTime = now + (1.0 / publishRateHz);

        float hMin = minHorizontalAngleDeg * Mathf.Deg2Rad;
        float hMax = maxHorizontalAngleDeg * Mathf.Deg2Rad;
        float vMin = minVerticalAngleDeg * Mathf.Deg2Rad;
        float vMax = maxVerticalAngleDeg * Mathf.Deg2Rad;

        int hStride = Mathf.Max(1, horizontalStride);
        int vStride = Mathf.Max(1, verticalStride);

        float hIncrement = horizontalSamples > 1 ? (hMax - hMin) / (horizontalSamples - 1) : 0f;
        float vIncrement = verticalSamples > 1 ? (vMax - vMin) / (verticalSamples - 1) : 0f;

        int estimated = ((horizontalSamples + hStride - 1) / hStride) * ((verticalSamples + vStride - 1) / vStride);
        List<Vector3> points = new List<Vector3>(estimated);
        QueryTriggerInteraction triggerInteraction = ignoreTriggerColliders ? QueryTriggerInteraction.Ignore : QueryTriggerInteraction.Collide;

        Vector3 origin = lidarFrame.position;
        Quaternion baseRotation = lidarFrame.rotation;

        for (int v = 0; v < verticalSamples; v += vStride)
        {
            float vAngleRad = vMin + (v * vIncrement);

            for (int h = 0; h < horizontalSamples; h += hStride)
            {
                float hAngleRad = hMin + (h * hIncrement);
                Quaternion localRotation = Quaternion.Euler(vAngleRad * Mathf.Rad2Deg, hAngleRad * Mathf.Rad2Deg, 0f);
                Vector3 direction = baseRotation * localRotation * Vector3.forward;

                if (Physics.Raycast(origin, direction, out RaycastHit hit, maxRange, layerMask, triggerInteraction))
                {
                    if (hit.distance < minRange)
                    {
                        continue;
                    }

                    Vector3 localPoint = lidarFrame.InverseTransformPoint(hit.point);
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
