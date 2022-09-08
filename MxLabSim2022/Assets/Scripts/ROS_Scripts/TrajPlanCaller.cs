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
    [HideInInspector] public bool qSent;

    private void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterRosService<TrajPlannerRequest, TrajPlannerResponse>(serviceName);
    }

    public void CallService()
    {
        TrajPlannerRequest req = new TrajPlannerRequest();
        req.x = target.transform.localPosition.x;
        req.y = target.transform.localPosition.y;
        req.z = target.transform.localPosition.z;

        ros.SendServiceMessage<TrajPlannerResponse>(serviceName, req, Callback);
    }

    void Callback(TrajPlannerResponse res)
    {
        res.ros[0] += 90;
        res.ros[1] += 90;

        ctrlClone.q = res.ros;
        qSent = true;
        Debug.LogWarning("Q recieved");
        //controller.q = res.ros;
    }
}
