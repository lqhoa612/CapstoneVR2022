using RosMessageTypes.Ur3UnityRos;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class JointStatePub : MonoBehaviour
{
    [InspectorReadOnly] public string topicName = "unity_joint_state";
    public CloneController cloneCtrl;
    public XRControllerCapture xrCapture;

    ROSConnection ros;

    private void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<JointPositionMsg>(topicName);
    }

    private void Update()
    {
        if (xrCapture.AisPressed == true)
        {
            JointPositionMsg jPos = new JointPositionMsg(cloneCtrl.GetJointAngles());
            jPos.joint_position[1] -= 90;
            ros.Publish(topicName, jPos);
        }
        
    }
}
