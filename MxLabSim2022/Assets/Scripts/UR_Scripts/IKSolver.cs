using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.HoaUnityRos;
using Unity.Robotics.ROSTCPConnector;

public class IKSolver : MonoBehaviour
{
    public string m_RosServiceName = "ur3_trajectory";
    public GameObject m_target;
    [HideInInspector] public float[] m_jointAngles;

    ROSConnection m_Ros;

    void Start()
    {
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterRosService<TrajectoryRequest, TrajectoryResponse>(m_RosServiceName);

        m_jointAngles = new float[] {0,0,0,0,0,0,0};
    }

    public void PublishJoints()
    {
        var request = new TrajectoryRequest();
        request.x = m_target.transform.position.x;
        request.y = m_target.transform.position.z;

        m_Ros.SendServiceMessage<TrajectoryResponse>(m_RosServiceName, request, TrajectoryResponse);
    }

    void TrajectoryResponse(TrajectoryResponse response)
    {
        Debug.Log("Trajectory returned");
        for (int i = 0; i < response.jointAngles.Length; i++)
        {
            m_jointAngles[i] = (float)response.jointAngles[i];
        }
    }
}
