using RosMessageTypes.Ur3UnityRos;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class TrajPlanCaller : MonoBehaviour
{
    ROSConnection ros;
    public string serviceName = "traj_planner";
    public GameObject target;
    public URController controller;
    bool ready = true;

    float delay = -1;

    private void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterRosService<TrajPlannerRequest, TrajPlannerResponse>(serviceName);
    }

    private void Update()
    {
        if (Time.time > delay && ready == true)
        {
            TrajPlannerRequest req = new TrajPlannerRequest();
            req.x = target.transform.localPosition.x;
            req.y = target.transform.localPosition.y;
            req.z = target.transform.localPosition.z;

            ros.SendServiceMessage<TrajPlannerResponse>(serviceName, req, Callback);
            delay = Time.time + 1.0f;
        }
    }

    void Callback(TrajPlannerResponse res)
    {
        ready = false;
        res.ros[0] += 90;
        res.ros[1] += 90;
        for (int i = 0; i < res.ros.Length; i++)
            res.ros[i] = Mathf.RoundToInt(res.ros[i]);
        controller.TrajExecute(res.ros);
    }
}
