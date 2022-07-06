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
    private float primaryTimer = 0;
    private float secondaryTimer = 0;

    private readonly XRNode xRNode = XRNode.RightHand;
    private readonly List<InputDevice> devices = new List<InputDevice>();
    private InputDevice device;

    private void GetDevice()
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
            //Debug.Log("TRIGGER");
            robotController.RotateJoint(indice, RotationDirection.Positive);
        }
        else if (device.TryGetFeatureValue(gripButtonUsage, out bool gripButtonValue) && gripButtonValue == true)
        {
            //Debug.Log("GRIP");
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

        if (device.TryGetFeatureValue(primaryButtonUsage, out bool primaryButtonValue))
        {
            if (primaryButtonValue == false)
                primaryTimer = 0;
            
            if (primaryTimer == 0 && primaryButtonValue == true)
            {
                primaryTimer += Time.deltaTime;
                //Debug.Log("PRIMARY" + primaryTimer);
                --indice;
            }
        }
        
        if (device.TryGetFeatureValue(secondaryButtonUsage, out bool secondaryButtonValue))
        {
            if (secondaryButtonValue == false)
                secondaryTimer = 0;

            if (secondaryTimer == 0 && secondaryButtonValue == true)
            {
                secondaryTimer += Time.deltaTime;
                //Debug.Log("SECOND" + secondaryTimer);
                ++indice;
            }
        }


        if (indice > 6) indice = 0;
        if (indice < 0) indice = 6;

    }

    public int GetControlledJoint()
    {
        return indice;
    }


}
