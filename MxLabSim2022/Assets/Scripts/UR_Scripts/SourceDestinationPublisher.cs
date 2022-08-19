using System;
using RosMessageTypes.Geometry;
using RosMessageTypes.Ur3Moveit;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.UrdfImporter;
using UnityEngine;

public class SourceDestinationPublisher : MonoBehaviour
{
    const int k_NumRobotJoints = 6;

    public static readonly string[] LinkNames =
        { "world/base_link/shoulder_link", "/upper_arm_link", "/forearm_link", "/wrist_1_link", "/wrist_2_link", "/wrist_3_link" };

    // Variables required for ROS communication
    [SerializeField]
    string m_TopicName = "/ur3";

    [SerializeField]
    GameObject m_NiryoOne;
    [SerializeField]
    GameObject m_Target;
    [SerializeField]
    GameObject m_TargetPlacement;
    readonly Quaternion m_PickOrientation = Quaternion.Euler(90, 90, 0);

    // Robot Joints
    UrdfJointRevolute[] m_JointArticulationBodies;

    // ROS Connector
    ROSConnection m_Ros;

    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterPublisher<UR3MoveitJointsMsg>(m_TopicName);

        m_JointArticulationBodies = new UrdfJointRevolute[k_NumRobotJoints];

        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += LinkNames[i];
            m_JointArticulationBodies[i] = m_NiryoOne.transform.Find(linkName).GetComponent<UrdfJointRevolute>();
        }
    }

    public void Publish()
    {
        var sourceDestinationMessage = new UR3MoveitJointsMsg();

        //for (var i = 0; i < k_NumRobotJoints; i++)
        //{
        //    sourceDestinationMessage.joints[i] = m_JointArticulationBodies[i].GetPosition();
        //}
        sourceDestinationMessage.joint_00 = m_JointArticulationBodies[0].GetPosition();
        sourceDestinationMessage.joint_01 = m_JointArticulationBodies[1].GetPosition();
        sourceDestinationMessage.joint_02 = m_JointArticulationBodies[2].GetPosition();
        sourceDestinationMessage.joint_03 = m_JointArticulationBodies[3].GetPosition();
        sourceDestinationMessage.joint_04 = m_JointArticulationBodies[4].GetPosition();
        sourceDestinationMessage.joint_05 = m_JointArticulationBodies[5].GetPosition();

        // Pick Pose
        sourceDestinationMessage.pick_pose = new PoseMsg
        {
            position = m_Target.transform.position.To<FLU>(),
            orientation = Quaternion.Euler(90, m_Target.transform.eulerAngles.y, 0).To<FLU>()
        };

        // Place Pose
        sourceDestinationMessage.place_pose = new PoseMsg
        {
            position = m_TargetPlacement.transform.position.To<FLU>(),
            orientation = m_PickOrientation.To<FLU>()
        };

        // Finally send the message to server_endpoint.py running in ROS
        m_Ros.Publish(m_TopicName, sourceDestinationMessage);
    }
}
