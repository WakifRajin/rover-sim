using System.Collections.Generic;
using RosMessageTypes.Geometry;
using RosMessageTypes.Tf2;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

[DisallowMultipleComponent]
public class RosTfPublisher : MonoBehaviour
{
    [System.Serializable]
    public class FrameLink
    {
        public string parentFrame = "base_link";
        public string childFrame = "imu_link";
        public Transform parentTransform;
        public Transform childTransform;
        public bool isStatic = true;
    }

    [SerializeField] private string tfTopic = "/tf";
    [SerializeField] private string tfStaticTopic = "/tf_static";
    [SerializeField] private string worldFrame = "map";
    [SerializeField] private string baseFrame = "base_link";
    [SerializeField] private Transform rootTransform;
    [SerializeField] private bool publishRootTransform = true;
    [SerializeField] private float publishRateHz = 30f;
    [SerializeField] private FrameLink[] frames;

    private ROSConnection ros;
    private double nextPublishTime;
    private bool publishedStatic;

    private void Awake()
    {
        if (rootTransform == null)
        {
            rootTransform = transform;
        }

        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TFMessageMsg>(tfTopic);
        ros.RegisterPublisher<TFMessageMsg>(tfStaticTopic);
    }

    private void Start()
    {
        PublishStaticFrames();
    }

    private void Update()
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
        PublishDynamicFrames(now);
    }

    private void PublishStaticFrames()
    {
        if (publishedStatic)
        {
            return;
        }

        double now = Time.timeAsDouble;
        List<TransformStampedMsg> transforms = new List<TransformStampedMsg>();

        if (frames != null)
        {
            foreach (FrameLink frame in frames)
            {
                if (frame == null || !frame.isStatic)
                {
                    continue;
                }

                if (!TryBuildTransform(frame, now, out TransformStampedMsg msg))
                {
                    continue;
                }

                transforms.Add(msg);
            }
        }

        if (transforms.Count > 0)
        {
            TFMessageMsg tfMsg = new TFMessageMsg();
            tfMsg.transforms = transforms.ToArray();
            ros.Publish(tfStaticTopic, tfMsg);
        }

        publishedStatic = true;
    }

    private void PublishDynamicFrames(double now)
    {
        List<TransformStampedMsg> transforms = new List<TransformStampedMsg>();

        if (publishRootTransform && rootTransform != null)
        {
            TransformStampedMsg rootMsg = BuildTransform(worldFrame, baseFrame, rootTransform.position, rootTransform.rotation, now);
            transforms.Add(rootMsg);
        }

        if (frames != null)
        {
            foreach (FrameLink frame in frames)
            {
                if (frame == null || frame.isStatic)
                {
                    continue;
                }

                if (!TryBuildTransform(frame, now, out TransformStampedMsg msg))
                {
                    continue;
                }

                transforms.Add(msg);
            }
        }

        if (transforms.Count > 0)
        {
            TFMessageMsg tfMsg = new TFMessageMsg();
            tfMsg.transforms = transforms.ToArray();
            ros.Publish(tfTopic, tfMsg);
        }
    }

    private bool TryBuildTransform(FrameLink frame, double now, out TransformStampedMsg msg)
    {
        msg = new TransformStampedMsg();
        if (frame.childTransform == null)
        {
            return false;
        }

        Transform parentTransform = frame.parentTransform;
        Vector3 localPos = parentTransform != null
            ? parentTransform.InverseTransformPoint(frame.childTransform.position)
            : frame.childTransform.position;

        Quaternion localRot = parentTransform != null
            ? Quaternion.Inverse(parentTransform.rotation) * frame.childTransform.rotation
            : frame.childTransform.rotation;

        msg = BuildTransform(frame.parentFrame, frame.childFrame, localPos, localRot, now);
        return true;
    }

    private TransformStampedMsg BuildTransform(string parentFrame, string childFrame, Vector3 localPosition, Quaternion localRotation, double time)
    {
        TransformStampedMsg msg = new TransformStampedMsg();
        msg.header = RosUnityConversions.CreateHeader(parentFrame, time);
        msg.child_frame_id = childFrame;

        TransformMsg transform = new TransformMsg();
        Vector3 rosPos = RosUnityConversions.UnityToRosVector(localPosition);
        Quaternion rosRot = RosUnityConversions.UnityToRosQuaternion(localRotation);

        transform.translation = new Vector3Msg(rosPos.x, rosPos.y, rosPos.z);
        transform.rotation = new QuaternionMsg(rosRot.x, rosRot.y, rosRot.z, rosRot.w);
        msg.transform = transform;

        return msg;
    }
}
