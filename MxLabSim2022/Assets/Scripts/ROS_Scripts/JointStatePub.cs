using RosMessageTypes.Ur3UnityRos;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class JointStatePub : MonoBehaviour
{
    [InspectorReadOnly] public string topicName = "unity_joint_state";
    public CloneController cloneCtrl;
    public XRControllerCapture xrCapture;
    [HideInInspector] public bool safeToPublish = true;

    ROSConnection ros;

    private void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<JointPositionMsg>(topicName);
    }

    private void Update()
    {
        //Debug.LogWarning(safeToPublish);
        if (xrCapture.AisPressed == true && safeToPublish == true)
        {
            JointPositionMsg jPos = new JointPositionMsg(cloneCtrl.GetJointAngles());
            jPos.joint_position[1] -= 90;
            ros.Publish(topicName, jPos);
        }
        
    }
}
