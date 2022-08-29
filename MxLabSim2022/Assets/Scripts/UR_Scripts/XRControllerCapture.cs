using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;
using Unity.Robotics;

public class XRControllerCapture : MonoBehaviour
{
    [HideInInspector]
    public bool AisPressed, BisPressed, XisPressed, YisPressed, gripLeft, gripRight, triggerLeft, triggerRight, menuIsPressed, joyRightPressed, joyLeftPressed;
    [HideInInspector]
    public float timer1, timer2, timer3, timer4;
    [HideInInspector] public int index;

    private InputDevice leftController;
    private InputDevice rightController;

    [HideInInspector]
    public Vector2 leftJoy = Vector2.zero, rightJoy = Vector2.zero;

    //public UR3TrajectoryPlanner m_publisher;
    //public SourceDestinationPublisher m_publisher;

    // Start is called before the first frame update
    void OnEnable()
    {
        index = 2;
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


        //Trigger
        if (leftController.TryGetFeatureValue(CommonUsages.triggerButton, out bool leftTriggerValue))
        {
            triggerLeft = leftTriggerValue;
            //Debug.Log("Left Trigger: " + leftTriggerValue);
        }
        if (rightController.TryGetFeatureValue(CommonUsages.triggerButton, out bool rightTriggerValue))
        {
            triggerRight = rightTriggerValue;

            if (rightTriggerValue == true)
            {
                //m_publisher.PublishJoints();
            }
            //Debug.Log("Right Trigger: " + rightTriggerValue);
        }


        //Grip
        if (leftController.TryGetFeatureValue(CommonUsages.gripButton, out bool leftGripValue))
        {
            gripLeft = leftGripValue;
            //Debug.Log("Left Grip: " + leftGripValue);
        }
        if (rightController.TryGetFeatureValue(CommonUsages.gripButton, out bool rightGripValue))
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
            AisPressed = isPressed2;
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
            BisPressed = isPressed4;
            //Debug.Log("B button: " + isPressed4);
        }


        //Joystick
        if (leftController.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 leftJoyValue))
        {
            //Debug.Log("Joystick Left: " + leftJoyValue);
            leftJoy = leftJoyValue;
        }
        if (rightController.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 rightJoyValue))
        {
            //Debug.Log("Joystick Right: " + rightJoyValue);
            rightJoy = rightJoyValue;
        }


        //Joystick Click
        if (leftController.TryGetFeatureValue(CommonUsages.primary2DAxisClick, out bool leftJoyIsPressed))
        {
            //Debug.Log("Joystick Left: " + leftJoyValue);
            joyLeftPressed = leftJoyIsPressed;
        }
        if (rightController.TryGetFeatureValue(CommonUsages.primary2DAxisClick, out bool rightJoyIsPressed))
        {
            //Debug.Log("Joystick Right: " + rightJoyValue);
            joyRightPressed = rightJoyIsPressed;
        }


        //Menu
        if (leftController.TryGetFeatureValue(CommonUsages.menuButton, out bool menuValue))
        {
            //Debug.Log("Menu: " + menuValue);
            menuIsPressed = menuValue;
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