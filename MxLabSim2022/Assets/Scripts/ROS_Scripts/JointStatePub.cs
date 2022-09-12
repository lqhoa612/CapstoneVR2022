using RosMessageTypes.Ur3UnityRos;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class JointStatePub : MonoBehaviour
{
    [InspectorReadOnly] public string topicName = "unity_joint_state";
    public URController urCtrl;
    public float publishMessageFrequency = .5f;

    ROSConnection ros;
    float timeElapsed;

    private void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<JointPositionMsg>(topicName);
    }

    private void Update()
    {
        timeElapsed += Time.deltaTime;
        if (timeElapsed > publishMessageFrequency)
        {
            JointPositionMsg jPos = new JointPositionMsg(urCtrl.GetJointAngles());
            jPos.joint_position[1] += 90;
            jPos.joint_position[3] += 90;
            ros.Publish(topicName, jPos);
            Debug.LogWarning("Message published.");
            timeElapsed = 0;
        }
    }
}
