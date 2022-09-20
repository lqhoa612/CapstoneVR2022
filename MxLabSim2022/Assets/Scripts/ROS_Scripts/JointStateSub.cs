using RosMessageTypes.Ur3UnityRos;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class JointStateSub : MonoBehaviour
{
    public string topicName = "lab_ur_joint_state";
    public URController urController;

    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<JointPositionMsg>(topicName, LabURCallback);
    }

    void LabURCallback(JointPositionMsg data)
    {
        var q_raw = data.joint_position;
        var q = q_raw;
        q[0] = q_raw[2];
        q[1] += 90;
        q[2] = -q_raw[0];
        urController.q = q;

        //for testing
        //Debug.LogWarning(q[0] + " | " + q[1] + " | " + q[2] + " | " + q[3] + " | " + q[4] + " | " + q[5]);
    }
}
