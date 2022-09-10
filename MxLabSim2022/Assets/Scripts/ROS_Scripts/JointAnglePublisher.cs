using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Ur3UnityRos;

public class JointAnglePublisher : MonoBehaviour
{
    public URController m_RobotController;

    ROSConnection ros;
    public string topicName = "joint_angles";
    // Publish every N seconds
    public float publishMessageFrequency = .5f;
    private float timeElapsed;

    // Start is called before the first frame update
    void Start()
    {
        // Start ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<JointAnglesMsg>(topicName);
    }

    // Update is called once per frame
    void Update()
    {
        timeElapsed += Time.deltaTime;
        if (timeElapsed > publishMessageFrequency)
        {
            JointAnglesMsg angles = new JointAnglesMsg(m_RobotController.GetJointAngles());
            ros.Publish(topicName, angles);
            Debug.LogWarning("OK");
            timeElapsed = 0;
        }
    }
}
