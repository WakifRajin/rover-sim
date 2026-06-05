using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

[DisallowMultipleComponent]
public class RosLidarPublisher : MonoBehaviour
{
    [SerializeField] private Transform lidarFrame;
    [SerializeField] private string topicName = "/scan";
    [SerializeField] private string frameId = "laser_link";
    [SerializeField] private float publishRateHz = 10f;
    [SerializeField] private int samples = 360;
    [SerializeField] private float minAngleDeg = -180f;
    [SerializeField] private float maxAngleDeg = 180f;
    [SerializeField] private float minRange = 0.1f;
    [SerializeField] private float maxRange = 30f;
    [SerializeField] private LayerMask layerMask = -1;
    [SerializeField] private bool ignoreTriggerColliders = true;

    private ROSConnection ros;
    private double nextPublishTime;

    private void Awake()
    {
        if (lidarFrame == null)
        {
            lidarFrame = transform;
        }

        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<LaserScanMsg>(topicName);
    }

    private void FixedUpdate()
    {
        if (publishRateHz <= 0f || samples <= 0)
        {
            return;
        }

        double now = Time.timeAsDouble;
        if (now < nextPublishTime)
        {
            return;
        }

        nextPublishTime = now + (1.0 / publishRateHz);

        float angleMinRad = minAngleDeg * Mathf.Deg2Rad;
        float angleMaxRad = maxAngleDeg * Mathf.Deg2Rad;
        float angleIncrement = samples > 1 ? (angleMaxRad - angleMinRad) / (samples - 1) : 0f;

        float[] ranges = new float[samples];
        Vector3 origin = lidarFrame.position;
        Quaternion baseRotation = lidarFrame.rotation;
        QueryTriggerInteraction triggerInteraction = ignoreTriggerColliders ? QueryTriggerInteraction.Ignore : QueryTriggerInteraction.Collide;

        for (int i = 0; i < samples; i++)
        {
            float angleRad = angleMinRad + (i * angleIncrement);
            Quaternion yaw = Quaternion.AngleAxis(angleRad * Mathf.Rad2Deg, Vector3.up);
            Vector3 direction = baseRotation * yaw * Vector3.forward;

            float range = float.PositiveInfinity;
            if (Physics.Raycast(origin, direction, out RaycastHit hit, maxRange, layerMask, triggerInteraction))
            {
                range = Mathf.Clamp(hit.distance, minRange, maxRange);
            }

            ranges[i] = range;
        }

        LaserScanMsg scan = new LaserScanMsg();
        scan.header = RosUnityConversions.CreateHeader(frameId, now);
        scan.angle_min = angleMinRad;
        scan.angle_max = angleMaxRad;
        scan.angle_increment = angleIncrement;
        scan.time_increment = (float)(1.0 / publishRateHz) / Mathf.Max(1, samples);
        scan.scan_time = (float)(1.0 / publishRateHz);
        scan.range_min = minRange;
        scan.range_max = maxRange;
        scan.ranges = ranges;
        scan.intensities = new float[0];

        ros.Publish(topicName, scan);
    }
}
