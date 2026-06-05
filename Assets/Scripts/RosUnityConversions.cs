using System;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using UnityEngine;

public static class RosUnityConversions
{
    public static Vector3 UnityToRosVector(Vector3 unity)
    {
        return new Vector3(unity.z, -unity.x, unity.y);
    }

    public static Quaternion UnityToRosQuaternion(Quaternion unity)
    {
        return new Quaternion(unity.z, -unity.x, unity.y, unity.w);
    }

    public static Vector3Msg ToRosVector3(Vector3 unity)
    {
        Vector3 ros = UnityToRosVector(unity);
        return new Vector3Msg(ros.x, ros.y, ros.z);
    }

    public static QuaternionMsg ToRosQuaternion(Quaternion unity)
    {
        Quaternion ros = UnityToRosQuaternion(unity);
        return new QuaternionMsg(ros.x, ros.y, ros.z, ros.w);
    }

    public static TimeMsg ToRosTime(double timeSeconds)
    {
        if (timeSeconds < 0.0)
        {
            timeSeconds = 0.0;
        }

        double secsDouble = Math.Floor(timeSeconds);
        if (secsDouble > int.MaxValue)
        {
            secsDouble = int.MaxValue;
        }

        int secs = (int)secsDouble;
        uint nsecs = (uint)Math.Floor((timeSeconds - secs) * 1e9);
        return new TimeMsg(secs, nsecs);
    }

    public static HeaderMsg CreateHeader(string frameId, double timeSeconds)
    {
        HeaderMsg header = new HeaderMsg();
        header.frame_id = frameId ?? string.Empty;
        header.stamp = ToRosTime(timeSeconds);
        return header;
    }
}
