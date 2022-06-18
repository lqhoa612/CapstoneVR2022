using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.XR;

public class NewHandController : MonoBehaviour
{
    [SerializeField] private NewHand hand;
    [SerializeField] private bool LeftHand, RightHand;
    private XRNode controller;
    private List<InputDevice> devices = new List<InputDevice>();
    private InputDevice device;

    void GetDevice()
    {
        InputDevices.GetDevicesAtXRNode(controller, devices);
        device = devices.FirstOrDefault();
    }

    private void Start()
    {
        if (LeftHand)
        {
            controller = XRNode.LeftHand;
        }
        else if (RightHand)
        {
            controller = XRNode.RightHand;
        }
        else
        {
            // do nothing
        }
    }

    private void OnEnable()
    {
        if (!device.isValid)
        {
            GetDevice();
        }
    }

    private void Update()
    {
        if (!device.isValid)
        {
            GetDevice();
        }

        InputFeatureUsage<float> gripUsage = CommonUsages.grip;
        //InputFeatureUsage<float> triggerUsage = CommonUsages.trigger;

        var gripParam = device.TryGetFeatureValue(gripUsage, out float gripValue);
        //var triggerParam = device.TryGetFeatureValue(triggerUsage, out float triggerValue);

        if (gripParam)
        {
            hand.SetGrip(gripValue);
        }
        //if (triggerParam) hand.setTrigger(triggerValue);

    }

}
