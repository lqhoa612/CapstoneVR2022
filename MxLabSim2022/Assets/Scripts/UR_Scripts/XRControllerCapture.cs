using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;
using Unity.Robotics;

public class XRControllerCapture : MonoBehaviour
{
    // Quest controller's properties
    [HideInInspector] public bool AisPressed, BisPressed, XisPressed, YisPressed, 
                                    leftGrip, rightGrip, leftTrigger, rightTrigger, 
                                    menuIsPressed, joyRightPressed, joyLeftPressed;
    [HideInInspector] public Vector2 leftJoy = Vector2.zero, rightJoy = Vector2.zero;
    [HideInInspector] public float leftGripF, rightGripF, leftTriggerF, rightTriggerF;

    private InputDevice leftController;
    private InputDevice rightController;

    //public UR3TrajectoryPlanner m_publisher;
    //public SourceDestinationPublisher m_publisher;

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

        //////////////////////////////////////////// Boolean //////////////////////////////////////////////////////
        //Trigger
        if (leftController.TryGetFeatureValue(CommonUsages.triggerButton, out bool leftTriggerPressed))
        {
            leftTrigger = leftTriggerPressed;
            //Debug.Log("Left Trigger: " + leftTriggerValue);
        }
        if (rightController.TryGetFeatureValue(CommonUsages.triggerButton, out bool rightTriggerPressed))
        {
            rightTrigger = rightTriggerPressed;
            //Debug.Log("Right Trigger: " + rightTriggerValue);
        }


        //Grip
        if (leftController.TryGetFeatureValue(CommonUsages.gripButton, out bool leftGripPressed))
        {
            leftGrip = leftGripPressed;
            //Debug.Log("Left Grip: " + leftGripValue);
        }
        if (rightController.TryGetFeatureValue(CommonUsages.gripButton, out bool rightGripPressed))
        {
            rightGrip = rightGripPressed;
            //Debug.Log("Right Grip: " + rightGripValue);
        }


        //A button
        if (leftController.TryGetFeatureValue(CommonUsages.primaryButton, out bool xPressed))
        {
            XisPressed = xPressed;
            //Debug.Log("X button: " + isPressed1);
        }
        if (rightController.TryGetFeatureValue(CommonUsages.primaryButton, out bool aPressed))
        {
            AisPressed = aPressed;
            //Debug.Log("A button: " + isPressed2);
        }


        //B button
        if (leftController.TryGetFeatureValue(CommonUsages.secondaryButton, out bool yPressed))
        {
            YisPressed = yPressed;
            //Debug.Log("Y button: " + isPressed3);
        }
        if (rightController.TryGetFeatureValue(CommonUsages.secondaryButton, out bool bPressed))
        {
            BisPressed = bPressed;
            //Debug.Log("B button: " + isPressed4);
        }


        //Joystick Click
        if (leftController.TryGetFeatureValue(CommonUsages.primary2DAxisClick, out bool leftJoyIsPressed))
        {
            joyLeftPressed = leftJoyIsPressed;
            //Debug.Log("Joystick Left: " + leftJoyValue);
        }
        if (rightController.TryGetFeatureValue(CommonUsages.primary2DAxisClick, out bool rightJoyIsPressed))
        {
            joyRightPressed = rightJoyIsPressed;
            //Debug.Log("Joystick Right: " + rightJoyValue);
        }


        //Menu
        if (leftController.TryGetFeatureValue(CommonUsages.menuButton, out bool menuPressed))
        {
            menuIsPressed = menuPressed;
            //Debug.Log("Menu: " + menuValue);
        }
        //////////////////////////////////////////// Boolean //////////////////////////////////////////////////////


        //////////////////////////////////////////// Vector2 //////////////////////////////////////////////////////
        //Joystick
        if (leftController.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 leftJoyValue))
        {
            leftJoy = leftJoyValue;
            //Debug.Log("Joystick Left: " + leftJoyValue);
        }
        if (rightController.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 rightJoyValue))
        {
            rightJoy = rightJoyValue;
            //Debug.Log("Joystick Right: " + rightJoyValue);
        }
        //////////////////////////////////////////// Vector2 //////////////////////////////////////////////////////


        //////////////////////////////////////////// Float ///////////////////////////////////////////////////////
        //Trigger
        if (leftController.TryGetFeatureValue(CommonUsages.trigger, out float leftTriggerValue))
        {
            leftTriggerF = leftTriggerValue;
            //Debug.Log("Left Trigger: " + leftTriggerValue);
        }
        if (rightController.TryGetFeatureValue(CommonUsages.trigger, out float rightTriggerValue))
        {
            rightTriggerF = rightTriggerValue;
            //Debug.Log("Right Trigger: " + rightTriggerValue);
        }


        //Grip
        if (leftController.TryGetFeatureValue(CommonUsages.grip, out float leftGripValue))
        {
            leftGripF = leftGripValue;
            //Debug.Log("Left Grip: " + leftGripValue);
        }
        if (rightController.TryGetFeatureValue(CommonUsages.grip, out float rightGripValue))
        {
            rightGripF = rightGripValue;
            //Debug.Log("Right Grip: " + rightGripValue);
        }
        //////////////////////////////////////////// Float ///////////////////////////////////////////////////////
        
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