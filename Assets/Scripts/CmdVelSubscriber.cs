using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class CmdVelSubscriber : MonoBehaviour
{
    [SerializeField] private ArticulationRoverController rover;
    [SerializeField] private string topicName = "/cmd_vel";
    [SerializeField] private bool useTwistStamped;

    void Awake()
    {
        if (rover == null)
        {
            Debug.LogError("[CmdVel] ArticulationRoverController reference is not assigned in the Inspector.");
        }
        else
        {
            Debug.Log($"[CmdVel] Subscribed to {topicName} → rover assigned: {rover.name}, enabled: {rover.enabled}, gameObject active: {rover.gameObject.activeInHierarchy}");
        }
        var ros = ROSConnection.GetOrCreateInstance();
        Debug.Log($"[CmdVel] ROSConnection instance: {ros?.RosIPAddress}:{ros?.RosPort}, connected: {ros?.HasConnectionThread}");
        if (useTwistStamped)
        {
            ros.Subscribe<TwistStampedMsg>(topicName, OnCmdVelStamped);
        }
        else
        {
            ros.Subscribe<TwistMsg>(topicName, OnCmdVel);
        }
    }

    void OnCmdVel(TwistMsg msg)
    {
        Debug.Log($"[CmdVel] lin.x={msg.linear.x:F2} ang.z={msg.angular.z:F2}");
        HandleCmdVel((float)msg.linear.x, (float)msg.angular.z);
    }

    void OnCmdVelStamped(TwistStampedMsg msg)
    {
        Debug.Log($"[CmdVel] lin.x={msg.twist.linear.x:F2} ang.z={msg.twist.angular.z:F2}");
        HandleCmdVel((float)msg.twist.linear.x, (float)msg.twist.angular.z);
    }

    void HandleCmdVel(float linearMetersPerSec, float angularRadPerSec)
    {
        if (rover == null)
        {
            return;
        }

        rover.SetCmdVel(linearMetersPerSec, angularRadPerSec);
    }
}