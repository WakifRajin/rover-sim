using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

[DisallowMultipleComponent]
public class RosImuPublisher : MonoBehaviour
{
    [SerializeField] private ArticulationBody rootBody;
    [SerializeField] private string topicName = "/imu/data";
    [SerializeField] private string frameId = "imu_link";
    [SerializeField] private float publishRateHz = 100f;
    [SerializeField] private bool includeGravity = true;
    [SerializeField] private bool useLocalFrame = true;

    private ROSConnection ros;
    private double nextPublishTime;
    private Vector3 lastVelocity;
    private bool hasLastVelocity;

    private void Awake()
    {
        if (rootBody == null)
        {
            rootBody = GetComponentInParent<ArticulationBody>();
        }

        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImuMsg>(topicName);
    }

    private void FixedUpdate()
    {
        if (publishRateHz <= 0f)
        {
            return;
        }

        double now = Time.timeAsDouble;
        if (now < nextPublishTime)
        {
            return;
        }

        nextPublishTime = now + (1.0 / publishRateHz);

        Vector3 velocity = rootBody != null ? rootBody.linearVelocity : Vector3.zero;
        Vector3 acceleration = hasLastVelocity
            ? (velocity - lastVelocity) / Time.fixedDeltaTime
            : Vector3.zero;

        lastVelocity = velocity;
        hasLastVelocity = true;

        if (includeGravity)
        {
            acceleration += -Physics.gravity;
        }

        Vector3 angularVelocity = rootBody != null ? rootBody.angularVelocity : Vector3.zero;

        if (useLocalFrame)
        {
            acceleration = transform.InverseTransformDirection(acceleration);
            angularVelocity = transform.InverseTransformDirection(angularVelocity);
        }

        ImuMsg msg = new ImuMsg();
        msg.header = RosUnityConversions.CreateHeader(frameId, now);
        msg.orientation = RosUnityConversions.ToRosQuaternion(transform.rotation);
        msg.angular_velocity = RosUnityConversions.ToRosVector3(angularVelocity);
        msg.linear_acceleration = RosUnityConversions.ToRosVector3(acceleration);

        MarkCovarianceUnknown(msg.orientation_covariance);
        MarkCovarianceUnknown(msg.angular_velocity_covariance);
        MarkCovarianceUnknown(msg.linear_acceleration_covariance);

        ros.Publish(topicName, msg);
    }

    private static void MarkCovarianceUnknown(double[] covariance)
    {
        if (covariance == null || covariance.Length == 0)
        {
            return;
        }

        covariance[0] = -1.0;
    }
}
