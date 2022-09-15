using UnityEngine;
using RosMessageTypes.Ur3UnityRos;
using Unity.Robotics.ROSTCPConnector;

public class TrajectoryServiceClient : MonoBehaviour
{
    ROSConnection ros;
    float freq = -1;

    public string serviceName = "traj_planner";
    public GameObject target;
    [HideInInspector] public float[] q = null;

    private void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterRosService<TrajectoryPlannerRequest, TrajectoryPlannerResponse>(serviceName);
    }

    private void Update()
    {
        if (Time.time > freq)
        {
            TrajectoryPlannerRequest request = new TrajectoryPlannerRequest();
            request.x = target.transform.localPosition.x;
            request.y = target.transform.localPosition.y;
            request.z = target.transform.localPosition.z;

            ros.SendServiceMessage<TrajectoryPlannerResponse>(serviceName, request, Callback);
            freq = Time.time + 5.0f;
        }
    }

    void Callback(TrajectoryPlannerResponse response)
    {
        response.q[0] += 90.0f;
        response.q[1] += 90.0f;

        q = response.q;
        Debug.Log("Response recieved.");
    }
}
