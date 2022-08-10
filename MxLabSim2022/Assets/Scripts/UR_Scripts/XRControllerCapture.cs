using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;
using Unity.Robotics;

public class XRControllerCapture : MonoBehaviour
{
    [HideInInspector]
    public bool AisPressed, BisPressed, XisPressed, YisPressed, gripLeft, gripRight, triggerLeft, triggerRight;
    [HideInInspector]
    public float timer1, timer2, timer3, timer4;
    [HideInInspector] public int index;

    public Unity.Robotics.UrdfImporter.Control.Controller manual;

    private InputDevice leftController;
    private InputDevice rightController;

    [HideInInspector]
    public Vector2 leftJoy = Vector2.zero, rightJoy = Vector2.zero;

    // Start is called before the first frame update
    void OnEnable()
    {
        if (!DevicesAreValid())
        {
            GetDevice();
        }

    }

    // Update is called once per frame
    void Update()
    {
        if (!DevicesAreValid())
        {
            GetDevice();
        }
        //manual.timerSI1 = timer4;
        //manual.timerSI2 = timer2;
        //manual.SelectionInput1 = BisPressed;
        //manual.SelectionInput2 = AisPressed;
        manual.moveDirection = rightJoy.x;
        manual.selectedIndex = index;


        //Trigger
        if (leftController.TryGetFeatureValue(CommonUsages.triggerButton, out bool leftTriggerValue) && leftTriggerValue  == true)
        {
            triggerLeft = leftTriggerValue;
            //Debug.Log("Left Trigger: " + leftTriggerValue);
        }
        if (rightController.TryGetFeatureValue(CommonUsages.triggerButton, out bool rightTriggerValue) && rightTriggerValue == true)
        {
            triggerRight = rightTriggerValue;
            //Debug.Log("Right Trigger: " + rightTriggerValue);
        }


        //Grip
        if (leftController.TryGetFeatureValue(CommonUsages.gripButton, out bool leftGripValue) && leftGripValue == true)
        {
            gripLeft = leftGripValue;
            //Debug.Log("Left Grip: " + leftGripValue);
        }
        if (rightController.TryGetFeatureValue(CommonUsages.gripButton, out bool rightGripValue) && rightGripValue == true)
        {
            gripRight = rightGripValue;
            //Debug.Log("Right Grip: " + rightGripValue);
        }


        //A button
        if (leftController.TryGetFeatureValue(CommonUsages.primaryButton, out bool isPressed1))
        {
            XisPressed = isPressed1;
            //Debug.Log("X button: " + isPressed1);
        }
        if (rightController.TryGetFeatureValue(CommonUsages.primaryButton, out bool isPressed2))
        {
            if (isPressed2 == false) timer2 = 0;
            if (isPressed2 == true && timer2 == 0)
            {
                timer2 += Time.deltaTime;
                index--;
                AisPressed = isPressed2;
            }
            //Debug.Log("A button: " + isPressed2);
        }


        //B button
        if (leftController.TryGetFeatureValue(CommonUsages.secondaryButton, out bool isPressed3))
        {
            YisPressed = isPressed3;
            //Debug.Log("Y button: " + isPressed3);
        }
        if (rightController.TryGetFeatureValue(CommonUsages.secondaryButton, out bool isPressed4))
        {
            if (isPressed4 == false) timer4 = 0;
            if (isPressed4 == true && timer4 == 0)
            {
                timer4 += Time.deltaTime;
                index++;
                BisPressed = isPressed4;
            }
            //Debug.Log("B button: " + isPressed4);
        }


        //Joystick
        if (leftController.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 leftJoyValue))
        {
            //Debug.Log("Joystick Left: " + leftJoyValue);
            leftJoy = leftJoyValue;
        }
        else
        {
            leftJoy = Vector2.zero;
        }
        if (rightController.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 rightJoyValue))
        {
            //Debug.Log("Joystick Right: " + rightJoyValue);
            rightJoy = rightJoyValue;
        }
        else
        {
            rightJoy = Vector2.zero;
        }
    }

    void GetDevice()
    {
        List<InputDevice> leftDevices = new List<InputDevice>();
        List<InputDevice> rightDevices = new List<InputDevice>();

        InputDeviceCharacteristics rightControllerCharacteristics = InputDeviceCharacteristics.Right | InputDeviceCharacteristics.Controller;
        InputDeviceCharacteristics leftControllerCharacteristics = InputDeviceCharacteristics.Left | InputDeviceCharacteristics.Controller;

        InputDevices.GetDevicesWithCharacteristics(rightControllerCharacteristics, rightDevices);
        InputDevices.GetDevicesWithCharacteristics(leftControllerCharacteristics, leftDevices);

        if (leftDevices.Count > 0)
        {
            leftController = leftDevices[0];
        }
        if (rightDevices.Count > 0)
        {
            rightController = rightDevices[0];
        }
    }

    bool DevicesAreValid()
    {
        return rightController.isValid && leftController.isValid;
    }
}