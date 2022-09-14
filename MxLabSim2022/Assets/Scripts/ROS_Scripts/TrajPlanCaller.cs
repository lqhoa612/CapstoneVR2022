using RosMessageTypes.Ur3UnityRos;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class TrajPlanCaller : MonoBehaviour
{
    ROSConnection ros;
    public string serviceName = "traj_planner";
    public GameObject target;
    [HideInInspector] public bool qSent;
    [HideInInspector] public float[] q = null;

    bool ready = true;

    private void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterRosService<TrajectoryPlannerRequest, TrajectoryPlannerResponse>(serviceName);
    }

    public void CallService()
    {
        if (ready == true)
        {
            TrajectoryPlannerRequest req = new TrajectoryPlannerRequest();
            req.x = target.transform.localPosition.x;
            req.y = target.transform.localPosition.y;
            req.z = target.transform.localPosition.z;

            ros.SendServiceMessage<TrajectoryPlannerResponse>(serviceName, req, Callback);
        }
        
    }

    void Callback(TrajectoryPlannerResponse res)
    {
        ready = false; // don't send again until ready
        if (res.q != null)
        {
            res.q[0] += 90;
            res.q[1] += 90;

            q = res.q;
        }
    }

}
