using RosMessageTypes.Ur3UnityRos;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class TrajPlanCaller : MonoBehaviour
{
    ROSConnection m_Ros;
    public string m_serviceName = "traj_planner";
    public GameObject m_target;
    public float[] new_q;

    float awaitingResponseUntilTimestamp = -1;

    private void Start()
    {
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterRosService<TrajPlannerRequest, TrajPlannerResponse>(m_serviceName);
    }

    private void Update()
    {
        if (Time.time > awaitingResponseUntilTimestamp)
        {
            TrajPlannerRequest req = new TrajPlannerRequest();
            req.x = m_target.transform.position.x;
            req.y = m_target.transform.position.y;
            req.z = m_target.transform.position.z;

            // Send message to ROS and return the response
            m_Ros.SendServiceMessage<TrajPlannerResponse>(m_serviceName, req, Callback);
            awaitingResponseUntilTimestamp = Time.time + 1.0f;// don't send again for 1 second, or until we receive a response
        }
    }

    void Callback(TrajPlannerResponse res)
    {
        awaitingResponseUntilTimestamp = -1;
        new_q = res.ros;
        Debug.Log("New q:" + new_q[0] + " | " + new_q[1] + " | " + new_q[2] + " | "
                            + new_q[3] + " | " + new_q[4] + " | " + new_q[5]);
    }
}
