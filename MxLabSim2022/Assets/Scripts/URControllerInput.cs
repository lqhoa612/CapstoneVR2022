using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.XR;


// Was named RightControllerInput
public class URControllerInput : MonoBehaviour
{
    public GameObject robot;
    private int indice = 0;

    private readonly XRNode xRNode = XRNode.RightHand;
    private readonly List<InputDevice> devices = new List<InputDevice>();
    private InputDevice device;

    void GetDevice()
    {
        InputDevices.GetDevicesAtXRNode(xRNode, devices);
        device = devices.FirstOrDefault();
    }

    void OnEnable()
    {
        if (!device.isValid)
            GetDevice();
    }

    void Update()
    {
        if (!device.isValid)
            GetDevice();


        RobotController robotController = robot.GetComponent<RobotController>();
        //indice = robotController.indices;

        // Rotate Joints
        InputFeatureUsage<bool> triggerButtonUsage = CommonUsages.triggerButton;
        InputFeatureUsage<bool> gripButtonUsage = CommonUsages.gripButton;

        if (device.TryGetFeatureValue(triggerButtonUsage, out bool triggerButtonValue) && triggerButtonValue == true)
        {
            Debug.Log("TRIGGER");
            robotController.RotateJoint(indice, RotationDirection.Positive);
        }
        else if (device.TryGetFeatureValue(gripButtonUsage, out bool gripButtonValue) && gripButtonValue == true)
        {
            Debug.Log("GRIP");
            robotController.RotateJoint(indice, RotationDirection.Negative);
        }
        else
        {
            //robotController.StopAllJointRotations();
            robotController.RotateJoint(indice, RotationDirection.None);
        }


        // Select Joints
        InputFeatureUsage<bool> primaryButtonUsage = CommonUsages.primaryButton;
        InputFeatureUsage<bool> secondaryButtonUsage = CommonUsages.secondaryButton;

        if (device.TryGetFeatureValue(primaryButtonUsage, out bool primaryButtonValue) && primaryButtonValue == true)
        {
            Debug.Log("PRIMARY");
            ++indice;
            for (int i = 0; i < 3; ++i);
        }
        else if (device.TryGetFeatureValue(secondaryButtonUsage, out bool secondaryButtonValue) && secondaryButtonValue == true)
        {
            Debug.Log("SECOND");
            --indice;
            for (int i = 0; i < 3; ++i) ;
        }


        if (indice > 6) indice = 0;
        if (indice < 0) indice = 6;


    }




}
