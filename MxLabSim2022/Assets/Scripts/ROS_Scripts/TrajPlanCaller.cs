using RosMessageTypes.Ur3UnityRos;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using System.Collections;
using System.Collections.Generic;
using System.Linq;

public class TrajPlanCaller : MonoBehaviour
{
    ROSConnection ros;
    public string serviceName = "traj_planner";
    public GameObject target;
    public URController ctrlUR3;
    public CloneController ctrlClone;
    [HideInInspector] public bool qSent;

    bool ready = true;
    float delay = 0.1f;

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
        //ready = false; // don't send again until ready
        //if (res.ros != null)
        //{
        //    res.ros[0] += 90;
        //    res.ros[1] += 90;
        //    ctrlUR3.ready = true;

        //    if (ctrlClone.ready == true && ctrlUR3.ready == true)
        //    {
        //        ctrlClone.q = res.ros;
        //    }else if (ctrlClone.ready == true && ctrlUR3.ready == false)
        //    {
        //        ready = true;
        //    }

        //}
        if (res.trajectory.Length > 0)
        {
            Debug.Log("Trajectory returned.");
            StartCoroutine(ExecuteTrajectory(res));
        }
        else
        {
            Debug.LogError("No trajectory returned from MoverService.");
        }
    }

    IEnumerator ExecuteTrajectory(TrajectoryPlannerResponse res)
    {
        if (res.trajectory != null)
        {
            foreach (var item in res.trajectory)
            {
                var jointPosition = item.positions;
                var result = jointPosition.Select(r => (float)r * Mathf.Rad2Deg).ToArray();

                // Set the joint values for every joint
                for (var joint = 0; joint < ctrlUR3.artiBodies.Length; joint++)
                {
                    var joint1XDrive = ctrlUR3.artiBodies[joint].xDrive;
                    joint1XDrive.target = result[joint];
                    ctrlUR3.artiBodies[joint].xDrive = joint1XDrive;
                }

                // Wait for robot to achieve pose for all joint assignments
                yield return new WaitForSeconds(delay);
            }
        }
    }
}
