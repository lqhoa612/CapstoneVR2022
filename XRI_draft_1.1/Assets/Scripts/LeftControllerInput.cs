using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.XR;

public class LeftControllerInput : MonoBehaviour
{
    public GameObject robot;
    private int indice = 0;

    private XRNode LeftHand = XRNode.LeftHand;
    private List<InputDevice> devices = new List<InputDevice>();
    private InputDevice device;

    void GetDevice()
    {
        InputDevices.GetDevicesAtXRNode(LeftHand, devices);
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

        // capturing trigger button
        bool triggerButtonValue = false;
        InputFeatureUsage<bool> triggerButtonUsage = CommonUsages.triggerButton;
        if (device.TryGetFeatureValue(triggerButtonUsage, out triggerButtonValue) && triggerButtonValue)
        {
            robotController.RotateJoint(indice, RotationDirection.Negative);
        }
        else robotController.StopAllJointRotations();

        // capturing primary button
        bool primaryButtonValue = false;
        InputFeatureUsage<bool> primaryButtonUsage = CommonUsages.primaryButton;
        if (device.TryGetFeatureValue(primaryButtonUsage, out primaryButtonValue) && primaryButtonValue)
        {
            if (primaryButtonValue == false) // debouncer
                --indice;
        }

        if (indice > 6) indice = 0;

        
    }

}
