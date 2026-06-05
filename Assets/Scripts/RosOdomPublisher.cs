using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Tf2;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

[DisallowMultipleComponent]
public class RosOdomPublisher : MonoBehaviour
{
    [SerializeField] private ArticulationBody rootBody;
    [SerializeField] private Transform baseLink;
    [SerializeField] private Transform odomFrame;
    [SerializeField] private string topicName = "/odom";
    [SerializeField] private string frameId = "odom";
    [SerializeField] private string childFrameId = "base_link";
    [SerializeField] private float publishRateHz = 30f;
    [SerializeField] private bool publishTf = true;
    [SerializeField] private string tfTopic = "/tf";

    private ROSConnection ros;
    private double nextPublishTime;
    private readonly double[] poseCovariance = new double[36];
    private readonly double[] twistCovariance = new double[36];

    private void Awake()
    {
        if (baseLink == null)
        {
            baseLink = transform;
        }

        if (rootBody == null)
        {
            rootBody = GetComponentInParent<ArticulationBody>();
        }

        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<OdometryMsg>(topicName);

        if (publishTf)
        {
            ros.RegisterPublisher<TFMessageMsg>(tfTopic);
        }

        MarkCovarianceUnknown(poseCovariance);
        MarkCovarianceUnknown(twistCovariance);
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

        Vector3 position = baseLink.position;
        Quaternion rotation = baseLink.rotation;
        if (odomFrame != null)
        {
            position = odomFrame.InverseTransformPoint(position);
            rotation = Quaternion.Inverse(odomFrame.rotation) * rotation;
        }

        Vector3 linearVelocity = rootBody != null ? rootBody.linearVelocity : Vector3.zero;
        Vector3 angularVelocity = rootBody != null ? rootBody.angularVelocity : Vector3.zero;
        if (baseLink != null)
        {
            linearVelocity = baseLink.InverseTransformDirection(linearVelocity);
            angularVelocity = baseLink.InverseTransformDirection(angularVelocity);
        }

        Vector3 rosPos = RosUnityConversions.UnityToRosVector(position);
        Quaternion rosRot = RosUnityConversions.UnityToRosQuaternion(rotation);
        Vector3 rosLinear = RosUnityConversions.UnityToRosVector(linearVelocity);
        Vector3 rosAngular = RosUnityConversions.UnityToRosVector(angularVelocity);

        PoseMsg pose = new PoseMsg(
            new PointMsg(rosPos.x, rosPos.y, rosPos.z),
            new QuaternionMsg(rosRot.x, rosRot.y, rosRot.z, rosRot.w));

        TwistMsg twist = new TwistMsg(
            new Vector3Msg(rosLinear.x, rosLinear.y, rosLinear.z),
            new Vector3Msg(rosAngular.x, rosAngular.y, rosAngular.z));

        OdometryMsg odom = new OdometryMsg();
        odom.header = RosUnityConversions.CreateHeader(frameId, now);
        odom.child_frame_id = childFrameId;
        odom.pose = new PoseWithCovarianceMsg(pose, poseCovariance);
        odom.twist = new TwistWithCovarianceMsg(twist, twistCovariance);

        ros.Publish(topicName, odom);

        if (publishTf)
        {
            TransformStampedMsg transform = BuildTransform(frameId, childFrameId, position, rotation, now);
            TFMessageMsg tfMsg = new TFMessageMsg();
            tfMsg.transforms = new TransformStampedMsg[] { transform };
            ros.Publish(tfTopic, tfMsg);
        }
    }

    private static void MarkCovarianceUnknown(double[] covariance)
    {
        if (covariance == null || covariance.Length == 0)
        {
            return;
        }

        covariance[0] = -1.0;
    }

    private static TransformStampedMsg BuildTransform(string parentFrame, string childFrame, Vector3 localPosition, Quaternion localRotation, double time)
    {
        TransformStampedMsg msg = new TransformStampedMsg();
        msg.header = RosUnityConversions.CreateHeader(parentFrame, time);
        msg.child_frame_id = childFrame;

        Vector3 rosPos = RosUnityConversions.UnityToRosVector(localPosition);
        Quaternion rosRot = RosUnityConversions.UnityToRosQuaternion(localRotation);

        TransformMsg transform = new TransformMsg();
        transform.translation = new Vector3Msg(rosPos.x, rosPos.y, rosPos.z);
        transform.rotation = new QuaternionMsg(rosRot.x, rosRot.y, rosRot.z, rosRot.w);
        msg.transform = transform;

        return msg;
    }
}
