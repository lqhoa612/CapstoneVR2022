using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.HoaUnityRos;

public class JointAnglePublisher : MonoBehaviour
{
    public Unity.Robotics.UrdfImporter.Control.Controller m_RobotController;

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
        ros.RegisterPublisher<JointAngleMsg>(topicName);
    }

    // Update is called once per frame
    void Update()
    {
        timeElapsed += Time.deltaTime;
        if (timeElapsed > publishMessageFrequency)
        {
            JointAngleMsg angles = new JointAngleMsg(m_RobotController.GetJointAngle());
            ros.Publish(topicName, angles);
            timeElapsed = 0;
        }
    }
}
