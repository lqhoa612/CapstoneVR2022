using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.XR;

public class XRControllerInput : MonoBehaviour
{
    public GameObject robot;
    private int indice = 0;

    //private readonly XRNode LeftHand = XRNode.LeftHand;
    //private InputDevice deviceLeft;
    private readonly XRNode RightHand = XRNode.RightHand;
    private InputDevice deviceRight;

    private readonly List<InputDevice> devices = new List<InputDevice>();
    
    void GetDevice()
    {
        //InputDevices.GetDevicesAtXRNode(LeftHand, devices);
        //deviceLeft = devices.First();
        InputDevices.GetDevicesAtXRNode(RightHand, devices);
        deviceRight = devices.FirstOrDefault();
    }

    void OnEnable()
    {
        //if (!deviceLeft.isValid)
        //    GetDevice();
        if (!deviceRight.isValid)
            GetDevice();
    }

    void Update()
    {
        //if (!deviceLeft.isValid)
        //    GetDevice();
        if (!deviceRight.isValid)
            GetDevice();

        RobotController robotController = robot.GetComponent<RobotController>();

        // capturing trigger button
        InputFeatureUsage<bool> triggerButtonUsage = CommonUsages.triggerButton;
        if (deviceRight.TryGetFeatureValue(triggerButtonUsage, out bool triggerButtonValue) && triggerButtonValue)
        {
            robotController.RotateJoint(indice, RotationDirection.Positive);
        } 
        else robotController.StopAllJointRotations();

        // capturing primary button
        InputFeatureUsage<bool> primaryButtonUsage = CommonUsages.primaryButton;
        if (deviceRight.TryGetFeatureValue(primaryButtonUsage, out bool primaryButtonValue) && primaryButtonValue)
        {
            if (primaryButtonValue == false) // debouncer
                ++indice;
        }



        //// capturing trigger button
        //InputFeatureUsage<bool> left_triggerButtonUsage = CommonUsages.triggerButton;
        //if (deviceLeft.TryGetFeatureValue(left_triggerButtonUsage, out bool left_triggerButtonValue) && left_triggerButtonValue)
        //{
        //    robotController.RotateJoint(indice, RotationDirection.Positive);
        //}
        //else robotController.StopAllJointRotations();

        //// capturing primary button
        //InputFeatureUsage<bool> left_primaryButtonUsage = CommonUsages.primaryButton;
        //if (deviceLeft.TryGetFeatureValue(left_primaryButtonUsage, out bool left_primaryButtonValue) && left_primaryButtonValue)
        //{
        //    if (left_primaryButtonValue == false) // debouncer
        //        ++indice;
        //}

        if (indice > 6) indice = 0;
    }

}
