using RosMessageTypes.Ur3UnityRos;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class JointPosPub : MonoBehaviour
{
    public URController ctrlUR;
    [InspectorReadOnly]public string topicName = "joint_pos";
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
            JointPositionMsg jPos = new JointPositionMsg(ctrlUR.GetJointAngles());
            ros.Publish(topicName, jPos);
            print("Message published.");
            timeElapsed = 0;
        }
    }
}
