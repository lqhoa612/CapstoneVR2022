using RosMessageTypes.Ur3UnityRos;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class TrajPlanCaller : MonoBehaviour
{
    ROSConnection ros;
    public string serviceName = "traj_planner";
    public GameObject target;
    public URController controller;
    public CloneController ctrlClone;

    private void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterRosService<TrajPlannerRequest, TrajPlannerResponse>(serviceName);
    }

    public void CallService()
    {
        ctrlClone.ToggleCloneMesh(true);

        TrajPlannerRequest req = new TrajPlannerRequest();
        req.x = target.transform.localPosition.x;
        req.y = target.transform.localPosition.y;
        req.z = target.transform.localPosition.z;

        ros.SendServiceMessage<TrajPlannerResponse>(serviceName, req, Callback);
    }

    void Callback(TrajPlannerResponse res)
    {
        if (res.ros == null) res.ros = ctrlClone.GetJointAngles();
        //if (res.ros == null) res.ros = controller.GetJointAngles();

        res.ros[0] += 90;
        res.ros[1] += 90;

        ctrlClone.q = res.ros;
        //controller.q = res.ros;
    }
}
