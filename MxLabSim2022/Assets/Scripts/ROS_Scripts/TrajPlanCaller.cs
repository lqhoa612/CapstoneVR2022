using RosMessageTypes.Ur3UnityRos;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class TrajPlanCaller : MonoBehaviour
{
    ROSConnection ros;
    public string serviceName = "traj_planner";
    public GameObject target;
    public URController ctrlUR3;
    public CloneController ctrlClone;
    [HideInInspector] public bool qSent;

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
            ctrlUR3.ready = true;

            if (ctrlClone.ready == true && ctrlUR3.ready == true)
            {
                ctrlClone.q = res.q;
            }
            else if (ctrlClone.ready == true && ctrlUR3.ready == false)
            {
                ready = true;
            }

        }
    }

}
