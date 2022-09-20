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
        if (data != null)
        {
            var q_raw = data.joint_position;
            var q = new float[] {   q_raw[2],
                                    q_raw[1] + 90,
                                    q_raw[0],
                                    q_raw[3],
                                    q_raw[4],
                                    q_raw[5] };
            urController.q = q;

            //for testing
            Debug.LogWarning(q[0] + " | " + q[1] + " | " + q[2] + " | " + q[3] + " | " + q[4] + " | " + q[5]);
        }
        else
        {
            Debug.LogError("No data from lab_ur_joint_state");
        }
    }
}
